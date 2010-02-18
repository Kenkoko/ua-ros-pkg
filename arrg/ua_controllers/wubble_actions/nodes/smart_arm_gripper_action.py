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
NAME = 'smart_arm_gripper_action'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionServer

from wubble_actions.msg import *
from std_msgs.msg import Float64
from pr2_controllers_msgs.msg import JointControllerState

import math


class SmartArmGripperAction():

    def __init__(self):

        # Initialize new node
        rospy.init_node(NAME, anonymous=True)

        # Initialize publisher & subscriber for left finger
        self.left_finger_frame = 'arm_left_finger_link'
        self.left_finger = JointControllerState(set_point=0.0, process_value=0.0, error=1.0)
        self.left_finger_pub = rospy.Publisher('finger_left_controller/command', Float64)
        rospy.Subscriber('finger_left_controller/state', JointControllerState, self.read_left_finger)
        rospy.wait_for_message('finger_left_controller/state', JointControllerState)

        # Initialize publisher & subscriber for right finger
        self.right_finger_frame = 'arm_right_finger_link'
        self.right_finger = JointControllerState(set_point=0.0, process_value=0.0, error=1.0)
        self.right_finger_pub = rospy.Publisher('finger_right_controller/command', Float64)
        rospy.Subscriber('finger_right_controller/state', JointControllerState, self.read_right_finger)
        rospy.wait_for_message('finger_right_controller/state', JointControllerState)

        # Initialize action server
        #self.result = SmartArmGripperResult()
        #self.feedback = SmartArmGripperFeedback()
        #self.feedback.gripper_position = [self.left_finger.process_value, self.right_finger.process_value]
        #self.server = SimpleActionServer("smart_arm_gripper_action", SmartArmGripperAction, self.execute_callback)

        # Initialize error & time thresholds
        self.ERROR_THRESHOLD = 0.005                        # Report success if error reaches below threshold
        self.TIMEOUT_THRESHOLD = rospy.Duration(15.0)       # Report failure if action does not succeed within timeout threshold

        # Reset gripper position
        #rospy.wait_for_service('pr2_controller_manager/switch_controller')
        rospy.sleep(1)
        self.reset_gripper_position()
        rospy.loginfo("%s: Ready to accept goals", NAME)


    def reset_gripper_position(self):
        self.left_finger_pub.publish(0.0)
        self.right_finger_pub.publish(0.0)
        rospy.sleep(5)


    def read_left_finger(self, data):
        self.left_finger = data
        self.has_latest_left_finger = True


    def read_right_finger(self, data):
        self.right_finger = data
        self.has_latest_right_finger = True


    def wait_for_latest_controller_states(self, timeout):
        self.has_latest_left_finger = False
        self.has_latest_right_finger = False
        r = rospy.Rate(100)
        start = rospy.Time.now()
        while (self.has_latest_left_finger == False or self.has_latest_right_finger == False) and \
                (rospy.Time.now() - start < timeout):
            r.sleep()


    def execute_callback(self, goal):
        r = rospy.Rate(100)
        #self.result.success = True

        # TODO
        
        #if (self.result.success):
        #    rospy.loginfo("%s: Goal Completed", NAME)
        #    self.wait_for_latest_controller_states(rospy.Duration(2.0))
        #    self.result.gripper_position = [self.left_finger.process_value, self.right_finger.process_value]
        #    self.server.set_succeeded(self.result)



if __name__ == '__main__':
    try:
        g = SmartArmGripperAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

