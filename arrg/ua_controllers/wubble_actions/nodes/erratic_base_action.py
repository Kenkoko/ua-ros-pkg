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

# Author: Anh Tran

PKG = 'wubble_actions'
NAME = 'erratic_base_action'

import roslib; roslib.load_manifest(PKG)
import rospy
import tf

from actionlib import SimpleActionServer
from actionlib import SimpleActionClient

from wubble_actions.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import Float64
from pr2_controllers_msgs.msg import JointControllerState

import math


class ErraticBaseActionServer():

    def __init__(self):

        # Initialize constants
        self.base_frame = '/base_footprint'

        # Initialize new node
        rospy.init_node(NAME, anonymous=True)

        # Initialize move_base action client
        self.move_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        # Initialize tf listener
        self.tf = tf.TransformListener()

        # Initialize erratic base action server
        self.result = ErraticBaseResult()
        self.feedback = ErraticBaseFeedback()
        self.server = SimpleActionServer(NAME, ErraticBaseAction, self.execute_callback)

        rospy.loginfo("%s: Ready to accept goals", NAME)



    def transform_target_point(self, point):
        self.tf.waitForTransform(self.base_frame, point.header.frame_id, rospy.Time(), rospy.Duration(5.0))
        return self.tf.transformPoint(self.base_frame, point)


    def move_to(self, target_pose):
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        goal.target_pose.header.stamp = rospy.Time.now()

        self.move_client.send_goal(goal, None, None, self.move_base_feedback_cb)
        
        while (not self.move_client.wait_for_result(rospy.Duration(0.01))):
            if self.server.is_preempt_requested():
                rospy.loginfo("%s: Aborted: Action Preempted", NAME)
                self.move_client.cancel_goal()
                return GoalStatus.PREEMPTED

        return self.move_client.get_state()


    def move_base_feedback_cb(self, fb):
        self.feedback.base_position = fb.base_position
        self.server.publish_feedback(self.feedback)


    def get_vicinity_target(self, target_pose, vicinity_range):
        vicinity_target = PoseStamped()
#        goal.target_pose.header.frame_id = frame_id
#        goal.target_pose.pose.position.x = x
#        goal.target_pose.pose.position.y = y
#        goal.target_pose.pose.orientation.w = 1.0

#        i = self.world_state.name.index("base_top_link")
#        wx = self.world_state.pose[i].position.x
#        wy = self.world_state.pose[i].position.y
#        wz = self.world_state.pose[i].position.z
#        vx = x - wx
#        vy = y - wy
#        vz = z - wz
#        vx *= 0.80
#        vy *= 0.80
#        vz *= 0.80
#        return [vx,vy,vz]

        return vicinity_target


    def execute_callback(self, goal):
        r = rospy.Rate(100)
        rospy.loginfo("%s: Executing move base", NAME)
        
        move_base_result = None

        if (goal.vicinity_range == 0.0):
            move_base_result = self.move_to(goal.target_pose)
        else:
            vicinity_target_pose = get_vicinity_target(goal.target_pose, goal.vicinity_range)
            move_base_result = self.move_to(vicinity_target_pose)
        
        if (move_base_result == GoalStatus.SUCCEEDED):
            self.result.base_position = self.feedback.base_position
            self.server.set_succeeded(self.result)
        elif (move_base_result == GoalStatus.PREEMPTED):
            self.server.set_preempted()
        else:
            self.server.set_aborted()


if __name__ == '__main__':
    try:
        w = ErraticBaseActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

