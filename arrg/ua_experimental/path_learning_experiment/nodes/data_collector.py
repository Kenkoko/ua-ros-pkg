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
from geometry_msgs.msg import PoseStamped, Pose, Point
from move_base_msgs.msg import MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Path

import random
import time
import math

import matplotlib.pyplot as plt

class Episode():
    def __init__(self, plan_waypoints, actual_waypoints):
        #self.goal_x = goal_pos.position.x
        #self.goal_y = goal_pos.position.y
        #self.start_x = start_pos.position.x
        #self.start_y = start_pos.position.y     

        self.planned_x = []
        self.planned_y = []
        for waypoint in plan_waypoints:
            self.planned_x += [waypoint.position.x]
            self.planned_y += [waypoint.position.y]

        self.current_x = []
        self.current_y = []
        for waypoint in actual_waypoints:
            self.current_x += [waypoint.position.x]
            self.current_y += [waypoint.position.y]    

    def plot(self):
        plt.clf()        
        plt.plot(self.current_x, self.current_y, 'go')
        plt.plot(self.planned_x, self.planned_y, 'bs')
        plt.axis([-3.5,3.5,-2.5,2.5])
        plt.draw()
        plt.savefig(str(rospy.Time.now()) + '.png')
        #plt.show()
        
               
class DataCollector():
    def __init__(self):
        self.is_running = True
        self.x_range = (-3.5,3.5)
        self.y_range = (-2.5,2.5)
        self.previous_success = True   

        self.episodes = []

        self.goal_pos = None
        self.start_x = [0]
        self.start_y = [0]     

        self.plan_waypoints = []    
        self.actual_waypoints = []

        self.old_status = None
    
        rospy.init_node('data_collector', anonymous=True)
    
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.handle_goal)
        rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.handle_feedback)
        rospy.Subscriber('move_base/status', GoalStatusArray, self.handle_status)
        rospy.Subscriber('move_base_node/NavfnROS/plan', Path, self.handle_plan)        

    def save_episode(self):
        e = Episode(self.plan_waypoints, self.actual_waypoints)
        self.episodes += [e]
        self.plan_waypoints = []
        self.actual_waypoints = []
        e.plot()

    def main_loop(self):
        while self.is_running:
            time.sleep(0.5)

    def handle_goal(self, msg):
        self.goal_pos = msg.pose

    def handle_status(self, msg):
        if msg.status_list:
            curr_status = msg.status_list[0].status
            print curr_status
            #if curr_status == 1:
            #    self.current_x = 
        
    def handle_feedback(self, msg):
        self.actual_waypoints += [msg.feedback.base_position.pose]

    def handle_plan(self, msg):
        if self.plan_waypoints:        
            self.save_episode()

        for pose_stamped in msg.poses:
            self.plan_waypoints += [pose_stamped.pose]

if __name__ == '__main__':
    try:
        collector = DataCollector()
        #t = Thread(target=collector.main_loop)
        #t.start()
        rospy.spin()
        collector.is_running = False
        #t.join()
    except rospy.ROSInterruptException: pass

