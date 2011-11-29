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

import math


class ErraticBaseActionServer():
    def __init__(self):
        self.base_frame = '/base_footprint'
        
        self.move_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()
        
        self.tf = tf.TransformListener()
        
        self.result = ErraticBaseResult()
        self.feedback = ErraticBaseFeedback()
        self.server = SimpleActionServer(NAME, ErraticBaseAction, self.execute_callback, auto_start=False)
        self.server.start()
        
        rospy.loginfo("%s: Ready to accept goals", NAME)


    def transform_target_point(self, point):
        self.tf.waitForTransform(self.base_frame, point.header.frame_id, rospy.Time(), rospy.Duration(5.0))
        return self.tf.transformPoint(self.base_frame, point)


    def move_to(self, target_pose):
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        goal.target_pose.header.stamp = rospy.Time.now()
        
        self.move_client.send_goal(goal=goal, feedback_cb=self.move_base_feedback_cb)
        
        while not self.move_client.wait_for_result(rospy.Duration(0.01)):
            # check for preemption
            if self.server.is_preempt_requested():
                rospy.loginfo("%s: Aborted: Action Preempted", NAME)
                self.move_client.cancel_goal()
                return GoalStatus.PREEMPTED
                
        return self.move_client.get_state()


    def move_base_feedback_cb(self, fb):
        self.feedback.base_position = fb.base_position
        if self.server.is_active():
            self.server.publish_feedback(self.feedback)


    def get_vicinity_target(self, target_pose, vicinity_range):
        vicinity_pose = PoseStamped()
        
        # transform target to base_frame reference
        target_point = PointStamped()
        target_point.header.frame_id = target_pose.header.frame_id
        target_point.point = target_pose.pose.position
        self.tf.waitForTransform(self.base_frame, target_pose.header.frame_id, rospy.Time(), rospy.Duration(5.0))
        target = self.tf.transformPoint(self.base_frame, target_point)
        
        rospy.logdebug("%s: Target at (%s, %s, %s)", NAME, target.point.x, target.point.y, target.point.z)
        
        # find distance to point
        dist = math.sqrt(math.pow(target.point.x, 2) + math.pow(target.point.y, 2))
        
        if (dist < vicinity_range):
            # if already within range, then no need to move
            vicinity_pose.pose.position.x = 0.0
            vicinity_pose.pose.position.y = 0.0
        else:
            # normalize vector pointing from source to target
            target.point.x /= dist
            target.point.y /= dist
            
            # scale normal vector to within vicinity_range distance from target
            target.point.x *= (dist - vicinity_range)
            target.point.y *= (dist - vicinity_range)
            
            # add scaled vector to source
            vicinity_pose.pose.position.x = target.point.x + 0.0
            vicinity_pose.pose.position.y = target.point.y + 0.0
            
        # set orientation
        ori = Quaternion()
        yaw = math.atan2(target.point.y, target.point.x)
        (ori.x, ori.y, ori.z, ori.w) = tf.transformations.quaternion_from_euler(0, 0, yaw)
        vicinity_pose.pose.orientation = ori
        
        # prep header
        vicinity_pose.header = target_pose.header
        vicinity_pose.header.frame_id = self.base_frame
        
        rospy.logdebug("%s: Moving to (%s, %s, %s)", NAME, vicinity_pose.pose.position.x, vicinity_pose.pose.position.y, vicinity_pose.pose.position.z)
        
        return vicinity_pose


    def execute_callback(self, goal):
        rospy.loginfo("%s: Executing move base", NAME)
        
        move_base_result = None
        
        if goal.vicinity_range == 0.0:
            # go to exactly
            move_base_result = self.move_to(goal.target_pose)
        else:
            # go near (within vicinity_range meters)
            vicinity_target_pose = self.get_vicinity_target(goal.target_pose, goal.vicinity_range)
            move_base_result = self.move_to(vicinity_target_pose)
            
        # check results
        if (move_base_result == GoalStatus.SUCCEEDED):
            rospy.loginfo("%s: Succeeded", NAME)
            self.result.base_position = self.feedback.base_position
            self.server.set_succeeded(self.result)
        elif (move_base_result == GoalStatus.PREEMPTED):
            rospy.loginfo("%s: Preempted", NAME)
            self.server.set_preempted()
        else:
            rospy.loginfo("%s: Aborted", NAME)
            self.server.set_aborted()


if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        w = ErraticBaseActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

