#!/usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
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

PKG = 'wubble_environments'

import roslib; roslib.load_manifest(PKG)

import rospy
from gazebo_plugins.msg import WorldState

def callback(data):
    rospy.loginfo(rospy.get_name()+' World State: ')
    for i in range(len(data.name)):
        pos = data.pose[i].position
        ori = data.pose[i].orientation
        lin = data.twist[i].linear
        ang = data.twist[i].angular
        frc = data.wrench[i].force
        trq = data.wrench[i].torque
        bbmin = data.boundsMin[i]
        bbmax = data.boundsMax[i]
        rospy.loginfo('\nName: %s',data.name[i])
        rospy.loginfo(' Pose: \n  Position: %s %s %s',pos.x,pos.y,pos.z)
        rospy.loginfo('  Orientation: %s %s %s %s',ori.x,ori.y,ori.z,ori.w)
        rospy.loginfo(' Twist: \n  Linear: %s %s %s',lin.x,lin.y,lin.z)
        rospy.loginfo('  Angular: %s %s %s',ang.x,ang.y,ang.z)
        rospy.loginfo(' Wrench: \n  Force: %s %s %s',frc.x,frc.y,frc.z)
        rospy.loginfo('  Torque: %s %s %s',trq.x,trq.y,trq.z)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('gazebo_world_state', WorldState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

