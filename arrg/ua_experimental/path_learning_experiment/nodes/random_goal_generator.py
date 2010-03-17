#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
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

PKG = 'path_learning_experiment'

import roslib; roslib.load_manifest(PKG)
import actionlib

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from move_base_msgs.msg import *

import random
import time
import math
from threading import Thread

# TODO: Gets stuck too often, maybe need to make the top round? Or have safety behavior/reset?

class RandomGoalGenerator():
    def __init__(self):
        self.is_running = True
        self.x_range = (-3.5,3.5)
        self.y_range = (-2.5,2.5)
        self.previous_success = True   
        self.current_goal = [0, 0]     
    
        rospy.init_node('random_goal_generator', anonymous=True)
    
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.handle_feedback)
        self.dist_pub = rospy.Publisher('distance_to_goal', Float64)

    def main_loop(self):
        while self.is_running:
            self.send_random_goal()
            time.sleep(0.5)

    def send_random_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        if self.previous_success:
            self.current_goal = [random.uniform(self.x_range[0], self.x_range[1]), random.uniform(self.y_range[0], self.y_range[1])]
            goal.target_pose.pose.position.x = self.current_goal[0]
            goal.target_pose.pose.position.y = self.current_goal[1]
        else:
            goal.target_pose.pose.position.x = 0
            goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.orientation.w = 1.0

        print "Sending goal: (", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, ")"
        self.client.send_goal(goal)
        self.previous_success = self.client.wait_for_result(rospy.Duration.from_sec(15.0))
        if self.previous_success:
            print "I DID IT!"    
        else:
            print "I FAILED! Trying to go home now..."
        
    def handle_feedback(self, feedback):
        current_pos = [feedback.feedback.base_position.pose.position.x, feedback.feedback.base_position.pose.position.y]
        distance = math.hypot(self.current_goal[0]-current_pos[0], self.current_goal[1]-current_pos[1])
        self.dist_pub.publish(distance)

if __name__ == '__main__':
    try:
        generator = RandomGoalGenerator()
        t = Thread(target=generator.main_loop)
        t.start()
        rospy.spin()
        generator.is_running = False
        t.join()
    except rospy.ROSInterruptException: pass

