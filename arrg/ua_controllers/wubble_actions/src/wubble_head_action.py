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

PKG = 'wubble_actions'
NAME = 'wubble_head_action'

import roslib; roslib.load_manifest(PKG)
import rospy
import tf

from actionlib import SimpleActionServer

from wubble_actions.msg import *
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, PointStamped
from pr2_controllers_msgs.msg import JointControllerState

import math


class WubblePointHeadControl():
    def __init__(self):

        # Initialize new node
        rospy.init_node(NAME, anonymous=True)

        # Initialize publisher & subscriber for pan
        self.head_pan_frame = 'head_pan_link' #'/stereo_optical_frame'
        self.head_pan = JointControllerState(process_value=0.0, error=1.0)
        self.head_pan_pub = rospy.Publisher('head_pan_controller/command', Float64)
        rospy.Subscriber('head_pan_controller/state', JointControllerState, self.read_current_pan)

        # Initialize publisher & subscriber for tilt
        self.head_tilt_frame = 'head_tilt_link' # '/stereo_optical_frame'
        self.head_tilt = JointControllerState(process_value=0.0, error=1.0)
        self.head_tilt_pub = rospy.Publisher('head_tilt_controller/command', Float64)
        rospy.Subscriber('head_tilt_controller/state', JointControllerState, self.read_current_tilt)

        # Initialize tf listener
        self.tf = tf.TransformListener()

        # Initialize action server
        self.result = WubblePointHeadResult()
        self.feedback = WubblePointHeadFeedback()
        self.feedback.head_position = [self.head_pan.process_value, self.head_tilt.process_value]
        self.server = SimpleActionServer("wubble_point_head_action", WubblePointHeadAction, self.execute_callback)

        # Initialize error & time thresholds
        self.ERROR_THRESHOLD = 0.02                     # Report success if error reaches below threshold
        self.TIMEOUT_THRESHOLD = rospy.Duration(5.0)    # Report failure if action does not succeed within timeout threshold


    def read_current_pan(self, data):
        self.head_pan = data


    def read_current_tilt(self, data):
        self.head_tilt = data


    def reset_head_position(self):
        self.head_pan.error = 1.0
        self.head_tilt.error = 1.0


    def calc_angle(self, x, y):
        if (x == 0 and y == 0):
            angle = 0.0        
        if (x > 0 and y == 0):
            angle = 0.0
        elif (x == 0 and y > 0):
            angle = math.pi / 2
        elif (x < 0 and y == 0):
            angle = math.pi
        elif (x == 0 and y < 0):
            angle = -(math.pi / 2)
        else:
            angle = math.atan2(y, x)
    
        return angle


    def transform_target_point(self, goal_point, goal_frame):
        pan_target_frame = self.head_pan_frame
        tilt_target_frame = self.head_tilt_frame
        ps = PointStamped()
        ps.header.frame_id = goal_frame
        ps.point = goal_point
        
        # Initialize a tf listener to retrieve tf info (time-out in 5 seconds)
        self.tf.waitForTransform(pan_target_frame, ps.header.frame_id, rospy.Time(), rospy.Duration(5.0))
        self.tf.waitForTransform(tilt_target_frame, ps.header.frame_id, rospy.Time(), rospy.Duration(5.0))

        # Transform target point to pan reference & retrieve the pan angle
        pan_target = self.tf.transformPoint(pan_target_frame, ps)
        pan_angle = self.calc_angle(pan_target.point.x, pan_target.point.y)
        rospy.loginfo("%s: Sucessfully transformed pan. Look at <%s, %s, %s> with angle %s", \
                NAME, pan_target.point.x, pan_target.point.y, pan_target.point.z, pan_angle)


        # Transform target point to tilt reference & retrieve the tilt angle
        tilt_target = self.tf.transformPoint(tilt_target_frame, ps)
        tilt_angle = self.calc_angle(tilt_target.point.x, tilt_target.point.z)
        rospy.loginfo("%s: Sucessfully transformed tilt. Look at <%s, %s, %s> with angle %s", \
                NAME, tilt_target.point.x, tilt_target.point.y, tilt_target.point.z, tilt_angle)

        return (pan_angle, tilt_angle)


    def execute_callback(self, goal):
        r = rospy.Rate(.1)
        self.result.success = True
        rospy.loginfo("%s: Executing, turning head towards the point <%s, %s, %s>", \
                        NAME, goal.point.x, goal.point.y, goal.point.z)

        try:
            # Try to convert target point to angles for pan & tilt
            (target_pan, target_tilt) = self.transform_target_point(goal.point, goal.frame_id)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("%s: Goal Failed, Unable to transform target point", NAME)
            self.result.success = False  
            self.result.head_position = self.feedback.head_position
            self.server.set_succeeded(self.result)
            return

	    # Publish goal command to controllers
        self.reset_head_position()
        self.head_pan_pub.publish(target_pan)
        self.head_tilt_pub.publish(target_pan)

        # Get time before starting action
        start_time = rospy.Time.now()

        while (self.head_pan.error > self.ERROR_THRESHOLD or self.head_tilt.error > self.ERROR_THRESHOLD):
		
	        # Cancel exe if another goal was received (i.e. preempt requested)
            if self.server.is_preempt_requested():
                rospy.loginfo("%s: Preempted", NAME)
                self.server.set_preempted()
                self.result.success = False
                break

            # Publish current head position as feedback
            self.feedback.head_position = [self.head_pan.process_value, self.head_tilt.process_value]
            self.server.publish_feedback(self.feedback)
            
            # Check timeout
            if (rospy.Time.now() - start_time > self.TIMEOUT_THRESHOLD):
                rospy.loginfo("%s: Action Timeout", NAME)
                self.result.success = False
                break

            r.sleep()

        if (self.result.success):
            rospy.loginfo("%s: Goal Completed", NAME)
        self.result.head_position = self.feedback.head_position
        self.server.set_succeeded(self.result)


if __name__ == '__main__':
    try:
        s = WubblePointHeadControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

