#!/usr/bin/env python

# Copyright (c) 2008, Willow Garage, Inc.  All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.  * Redistributions
#     in binary form must reproduce the above copyright notice, this list of
#     conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution. # Neither the name of
#     the Willow Garage, Inc. nor the names of its contributors may be used to
#     endorse or promote products derived from this software without specific
#     prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Anh Tran
# Auther: Jeremy Wright

PKG = 'wubble_environments'
NAME = 'icarus_world_state_server'

import roslib; roslib.load_manifest(PKG)
import rospy
import re
import math

from gazebo_plugins.msg import WorldState
from wubble_environments.srv import *
from std_msgs.msg import String

class Vector3f():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class GazeboWorldObject():
    def __init__(self, sizex, sizey, sizez):
        self.size = Vector3f(sizex, sizey, sizez)
        self.s_radius = math.sqrt(sizex * sizex + sizey * sizey + sizez * sizez) / 2

class GazeboDescriptionParser():
    def __init__(self, world_description, robot_description):
        self.gazebo_world = dict()
        self.gazebo_world = dict(self.gazebo_world.items() + self.parse_world(world_description).items())
        self.gazebo_world = dict(self.gazebo_world.items() + self.parse_robot(robot_description).items())
        
    def parse_world(self, world_description):
        world = dict()
        regexbody = re.compile(r"<(?P<tag>body:.*?) .*?name *?= *?['\"](.*?)['|\"] *?>(.*?)< *?/(?P=tag) *?>", re.DOTALL)
        regexsize = re.compile(r"< *?visualsize *?>(.*?)< *?/visualsize *?>", re.DOTALL)
        for matchbody in re.finditer(regexbody, world_description):
            matchsize = re.search(regexsize, matchbody.group(3))
            if (matchsize):
                size = matchsize.group(1).strip().split()
                while len(size) < 3:
                    size.append('0')
                world[matchbody.group(2).strip()] = GazeboWorldObject(float(size[0]), float(size[1]), float(size[2]))
        return world
        
    def parse_robot(self, robot_description):
        robot = dict()
        regexlink = re.compile(r"<link .*?name *?= *?['\"](.*?)['|\"] *?>(.*?)< *?/link *?>", re.DOTALL)
        regexsize = re.compile(r"size= *?['\"](.*?)['|\"] *?/>", re.DOTALL)
        for matchlink in re.finditer(regexlink, robot_description):
            # Publish only the base_link as the representation for the entire robot
            if matchlink.group(1).strip() == 'base_link':
                matchsize = re.search(regexsize, matchlink.group(2))
                if matchsize:
                    size = matchsize.group(1).strip().split()
                    while len(size) < 3:
                        size.append('0')
                    robot[matchlink.group(1).strip()] = GazeboWorldObject(float(size[0]), float(size[1]), float(size[2]))
                break;
        return robot
        
    def has_object(self, name):
        return name in self.gazebo_world
        
    def get_object(self, name):
        return self.gazebo_world.get(name, None)

class IcarusWorldStateServer():
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        # Set gazebo_world as None to be retrieved in handler
        self.gazebo_world = None
        
        # Start get_icarus_world_state service
        rospy.Service('get_icarus_world_state', IcarusWorldState, self.get_icarus_world_state_handler)
        
        # Start gazebo_world_state subscriber
        rospy.Subscriber('gazebo_world_state', WorldState, self.gazebo_world_state_handler)
        
        # Start icarus_world_state publisher
        self.icarus_world_state_pub = rospy.Publisher('icarus_world_state', String)
        self.icarus_world_state = '\'()'
        
    def get_icarus_world_state_handler(self, req):
        return IcarusWorldStateResponse(self.icarus_world_state)
        
    def gazebo_world_state_handler(self, data):
        # Retrieve the static world & robot descriptions once on first run
        if self.gazebo_world == None:
            world_param = rospy.search_param('gazebo_world_description')
            world_description = rospy.get_param(world_param, '')
            robot_param = rospy.search_param('robot_description')
            robot_description = str(rospy.get_param(robot_param, ''))
            self.gazebo_world = GazeboDescriptionParser(world_description, robot_description)
            
        # Begin parsing icarus world state
        icarus_state = '(list'
        regex = re.compile('(env|obj)_(.*?)_(.*?)_(.*?)_body')
        
        for i in range(len(data.name)):
            pos = data.pose[i].position
            ori = data.pose[i].orientation
            lin = data.twist[i].linear
            ang = data.twist[i].angular
            frc = data.wrench[i].force
            trq = data.wrench[i].torque
            
            # Only publish recognizable objects from the world & robot desscriptions
            if self.gazebo_world != None and self.gazebo_world.has_object(data.name[i].strip()):
                match = re.search(regex, data.name[i])
                
                if match:
                    if match.group(1) == 'env':
                        icarus_state += ' \'(environment %s' % (data.name[i])
                    else:
                        icarus_state += ' \'(object %s' % (data.name[i])
                    icarus_state += ' type %s' % (match.group(2))
                    icarus_state += ' color %s' % (match.group(4))
                else:
                    icarus_state += ' \'(object %s' % (data.name[i])
                    if data.name[i].strip() == 'base_link':
                        icarus_state += ' type self'
                    else:
                        icarus_state += ' type empty'
                    icarus_state += ' color empty'
                    
                icarus_state += ' role empty'
                icarus_state += ' status empty'
                icarus_state += ' xpos %s ypos %s zpos %s' % (pos.x, pos.y, pos.z)
                icarus_state += ' xori %s yori %s zori %s wori %s' % (ori.x, ori.y, ori.z, ori.w)
                icarus_state += ' xlin %s ylin %s zlin %s' % (lin.x, lin.y, lin.z)
                icarus_state += ' xang %s yang %s zang %s' % (ang.x, ang.y, ang.z)
                icarus_state += ' xtrq %s ytrq %s ztrq %s' % (trq.x, trq.y, trq.z)
                
                obj = self.gazebo_world.get_object(data.name[i].strip())
                icarus_state += ' xsiz %s ysiz %s zsiz %s srad %s)' % (obj.size.x, obj.size.y, obj.size.z, obj.s_radius)
                
        icarus_state += ')'
        
        self.icarus_world_state = icarus_state
        self.icarus_world_state_pub.publish(icarus_state)


if __name__ == '__main__':
    try:
        s = IcarusWorldStateServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

