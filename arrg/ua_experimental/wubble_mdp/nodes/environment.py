#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
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

# Author: Daniel Hewlett

PKG = 'wubble_mdp'
NAME = 'environment'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionClient

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import *
from simulator_state.msg import *
from wubble_mdp.srv import *

from math import pi
import numpy
from tf.transformations import quaternion_about_axis, euler_from_quaternion

from wubble_mdp.state import *
from wubble_mdp.relations import *

class Environment():

    def __init__(self):
        rospy.init_node(NAME)
    
        self.mdp_state = {} # map from object name to tuple
        self.gz_relations = []
        self.last_mdp_state = {}
        self.class_map = {}
        self.delta = 0.5
        
        # Subscribe to the raw gazebo world state
        self.state_subscriber = rospy.Subscriber('gazebo_world_state', WorldState, self.convert_state)
        # Interface with move_base for navigation
        self.move_client = SimpleActionClient('move_base', MoveBaseAction)

        # Services        
        self.action_server = rospy.Service('environment/perform_action', PerformAction, self.perform_action)
        self.action_server = rospy.Service('environment/get_state', GetState, self.get_state)        
        self.action_server = rospy.Service('environment/simulate_action', SimulateAction, self.simulate_action)

    def get_state(self, req):   
        res = GetStateResponse()
        res.state.object_states = self.create_state_msg() 
        res.state.relations = compute_initial_relations(self.mdp_state, self.class_map)
        #res.state.relations += self.gz_relations # Disabling gz_relations for now
        
        rospy.loginfo("get_state response:\n%s", res)
        
        return res

    def perform_action(self, req):
        #print "BEFORE: ", self.get_robot_state()

        # Now we should always have a last state
        
        if len(self.last_mdp_state) == 0:
            self.create_state_msg()
        
        before_state = self.last_mdp_state
        
        # TODO: Get the "before" state
        
        res = PerformActionResponse()
        
        curr_state = self.get_robot_state() # just to be safe

        if not curr_state:
            return None

        new_pos = (curr_state[0], curr_state[1]) # remember slices?
        new_theta = curr_state[2]

        if req.action == 'forward':
            new_pos = self.compute_new_position(curr_state)
            new_theta = canonical_angles[new_theta]
        elif req.action == 'left' or req.action == 'right':
            new_theta = self.compute_new_theta(new_theta, req.action)
        else:
            rospy.logerr("%s is not a valid action!", req.action)
            return None
        
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.pose.position.x = new_pos[0]
        goal.target_pose.pose.position.y = new_pos[1]
        q = quaternion_about_axis(new_theta, (0, 0, 1)) 
        goal.target_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # print goal

        # wait for 5 seconds, then abandon ship
        # Actually, either way we just return the state, so whatever
        self.move_client.send_goal(goal, None, None, None)
        if self.move_client.wait_for_result(rospy.Duration(5.0)):
            print "ALL GOOD" #TODO: Need to forward this error on somehow?
        else:
            print "GOT PROBLEMS"

        #res.state.relations = compute_relations(self.mdp_state, self.last_mdp_state)
        res.state.object_states = self.create_state_msg()  
        res.state.relations = compute_relations(self.mdp_state, before_state, self.class_map)
        #res.state.relations += self.gz_relations
              
        return res

    def simulate_action(self, req):
        temp = mdp_state_from_msg(req.state)
        class_map = make_class_map(req.state)
        curr_state = temp[0]
        last_state = temp[1]

        print curr_state
        print last_state

        # TODO: REPLACE THIS WITH A FUNCTION THAT FINDS THE THING WITH CLASS ROBOT
        robot_state = curr_state[get_robot_name(class_map)]
        new_pos = (robot_state[0], robot_state[1]) # remember slices?
        new_theta = robot_state[2]

        if req.action == 'forward':
            new_pos = self.compute_new_position(robot_state)
            new_theta = canonical_angles[new_theta]
        elif req.action == 'left' or req.action == 'right':
            new_theta = self.compute_new_theta(new_theta, req.action)
        elif req.action == 'wait':
            new_theta = canonical_angles[new_theta]
        else:
            rospy.logerr("%s is not a valid action!", req.action)
            return None
        
        new_state = {}
        for name, state in curr_state.items():
            if class_map[name] == 'robot': # assuming for now that only the robot moves
                new_state[name] = (new_pos[0], new_pos[1], convert_yaw(new_theta))
            else:
                new_state[name] = state
        
        res = SimulateActionResponse()
        res.state.object_states = []
        for name, state in new_state.items():
            res.state.object_states.append(make_obj_state(name, state, curr_state[name], class_map))
        res.state.relations = compute_relations(new_state, curr_state, class_map, simulation=True)
        
        # TODO: How do we handle Contact? Must have a heuristic of some kind
        #res.state.relations += self.gz_relations

        return res
    
    def create_state_msg(self):
        curr_state = self.mdp_state
        last_state = {}

        states = []
        for name, state in curr_state.items():
            states.append(make_obj_state(name, state, lookup(name, self.last_mdp_state), self.class_map))
            last_state[name] = state # Store the state into the last state

        self.last_mdp_state = last_state
                
        return states

    def compute_new_theta(self, old_theta, action):
        index = angle_list.index(old_theta)
        new_or = ''
        new_angle = 0
        if action == 'right':
            new_or = angle_list[(index+1)%8]
        else:
            new_or = angle_list[(index-1)%8]
        new_angle = canonical_angles[new_or]
        print old_theta, new_or, new_angle
        return new_angle

    def convert_state(self, gz_state):
        """Given a WorldState msg, returns an MDPState msg"""
        new_state = {}
        new_class_map = {}
        self.gz_relations = gz_state.simple_relations
        for obj_info in gz_state.object_info:
            x = round_to_delta(obj_info.pose.position.x, self.delta)
            y = round_to_delta(obj_info.pose.position.y, self.delta)
            q = [obj_info.pose.orientation.x, obj_info.pose.orientation.y, 
                 obj_info.pose.orientation.z, obj_info.pose.orientation.w]
            theta = convert_yaw(euler_from_quaternion(q)[2])
            new_state[obj_info.name] = (x, y, theta)
            new_class_map[obj_info.name] = infer_class_from_name(obj_info.name)
            
        self.mdp_state = new_state
        self.class_map = new_class_map

    def compute_new_position(self, robot_state):
        """This is the transition function for the robot state"""
        
        print "THIS: ", robot_state

        new_x = robot_state[0]
        new_y = robot_state[1]
        orientation = robot_state[2]

        if orientation.find('N') >= 0:
            new_y += self.delta
        elif orientation.find('S') >= 0:
            new_y -= self.delta

        if orientation.find('E') >= 0:
            new_x += self.delta
        elif orientation.find('W') >= 0:
            new_x -= self.delta

        return (new_x, new_y)

    def get_robot_state(self):
        return lookup('robot_description', self.mdp_state,)

if __name__ == "__main__":
    e = Environment()
    rospy.loginfo('Environment Ready.')
    rospy.spin()











