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
NAME = 'smart_arm_action'

import roslib; roslib.load_manifest(PKG)
import rospy
import tf

from actionlib import SimpleActionServer

from wubble_actions.msg import *
from smart_arm_kinematics.srv import SmartArmIK
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from pr2_controllers_msgs.msg import JointControllerState

import math


class SmartArmActionServer():

    def __init__(self):

        # Initialize constants
        self.JOINTS_COUNT = 4                           # Number of joints to manage
        self.ERROR_THRESHOLD = 0.15                     # Report success if error reaches below threshold
        self.TIMEOUT_THRESHOLD = rospy.Duration(15.0)   # Report failure if action does not succeed within timeout threshold

        # Initialize new node
        rospy.init_node(NAME + 'server', anonymous=True)

        # Initialize publisher & subscriber for shoulder pan
        self.shoulder_pan_frame = 'arm_shoulder_pan_link'
        self.shoulder_pan = JointControllerState(set_point=0.0, process_value=0.0, error=1.0)
        self.shoulder_pan_pub = rospy.Publisher('shoulder_pan_controller/command', Float64)
        rospy.Subscriber('shoulder_pan_controller/state', JointControllerState, self.read_shoulder_pan)
        rospy.wait_for_message('shoulder_pan_controller/state', JointControllerState)

        # Initialize publisher & subscriber for shoulder tilt
        self.shoulder_tilt_frame = 'arm_shoulder_tilt_link'
        self.shoulder_tilt = JointControllerState(set_point=0.0, process_value=0.0, error=1.0)
        self.shoulder_tilt_pub = rospy.Publisher('shoulder_tilt_controller/command', Float64)
        rospy.Subscriber('shoulder_tilt_controller/state', JointControllerState, self.read_shoulder_tilt)
        rospy.wait_for_message('shoulder_tilt_controller/state', JointControllerState)

        # Initialize publisher & subscriber for elbow tilt
        self.elbow_tilt_frame = 'arm_elbow_tilt_link'
        self.elbow_tilt = JointControllerState(set_point=0.0, process_value=0.0, error=1.0)
        self.elbow_tilt_pub = rospy.Publisher('elbow_tilt_controller/command', Float64)
        rospy.Subscriber('elbow_tilt_controller/state', JointControllerState, self.read_elbow_tilt)
        rospy.wait_for_message('elbow_tilt_controller/state', JointControllerState)

        # Initialize publisher & subscriber for shoulder tilt
        self.wrist_rotate_frame = 'arm_wrist_rotate_link'
        self.wrist_rotate = JointControllerState(set_point=0.0, process_value=0.0, error=1.0)
        self.wrist_rotate_pub = rospy.Publisher('wrist_rotate_controller/command', Float64)
        rospy.Subscriber('wrist_rotate_controller/state', JointControllerState, self.read_wrist_rotate)
        rospy.wait_for_message('wrist_rotate_controller/state', JointControllerState)

        # Initialize tf listener
        self.tf = tf.TransformListener()

        # Initialize joints action server
        self.result = SmartArmResult()
        self.feedback = SmartArmFeedback()
        self.feedback.arm_position = [self.shoulder_pan.process_value, self.shoulder_tilt.process_value, \
                    self.elbow_tilt.process_value, self.wrist_rotate.process_value]
        self.server = SimpleActionServer(NAME, SmartArmAction, self.execute_callback)

        # Reset arm position
        rospy.sleep(1)
        self.reset_arm_position()
        rospy.loginfo("%s: Ready to accept goals", NAME)


    def reset_arm_position(self):
        # reset arm to cobra position
        self.shoulder_pan_pub.publish(0.0)
        self.shoulder_tilt_pub.publish(1.972222)
        self.elbow_tilt_pub.publish(-1.972222)
        self.wrist_rotate_pub.publish(0.0)
        rospy.sleep(12)


    def read_shoulder_pan(self, pan_data):
        self.shoulder_pan = pan_data
        self.has_latest_shoulder_pan = True


    def read_shoulder_tilt(self, tilt_data):
        self.shoulder_tilt = tilt_data
        self.has_latest_shoulder_tilt = True


    def read_elbow_tilt(self, tilt_data):
        self.elbow_tilt = tilt_data
        self.has_latest_elbow_tilt = True


    def read_wrist_rotate(self, rotate_data):
        self.wrist_rotate = rotate_data
        self.has_latest_wrist_rotate = True


    def wait_for_latest_controller_states(self, timeout):
        self.has_latest_shoulder_pan = False
        self.has_latest_shoulder_tilt = False
        self.has_latest_elbow_tilt = False
        self.has_latest_wrist_rotate = False
        r = rospy.Rate(100)
        start = rospy.Time.now()
        while (self.has_latest_shoulder_pan == False or self.has_latest_shoulder_tilt == False or \
                self.has_latest_elbow_tilt == False or self.has_latest_wrist_rotate == False) and \
                (rospy.Time.now() - start < timeout):
            r.sleep()


    def transform_target_point(self, point):
        rospy.loginfo("%s: Retrieving IK solutions", NAME)
        rospy.wait_for_service('smart_arm_ik_service', 10)
        ik_service = rospy.ServiceProxy('smart_arm_ik_service', SmartArmIK)
        resp = ik_service(point)
        if (resp and resp.success):
            return resp.solutions[0:4]
        else:
            raise Exception, "Unable to obtain IK solutions."


    def execute_callback(self, goal):
        r = rospy.Rate(100)
        self.result.success = True
        self.result.arm_position = [self.shoulder_pan.process_value, self.shoulder_tilt.process_value, \
                self.elbow_tilt.process_value, self.wrist_rotate.process_value]
        rospy.loginfo("%s: Executing move arm", NAME)
        
        # Initialize target joints
        target_joints = list()
        for i in range(self.JOINTS_COUNT):
            target_joints.append(0.0)
        
        # Retrieve target joints from goal
        if (len(goal.target_joints) > 0):
            for i in range(min(len(goal.target_joints), len(target_joints))):
                target_joints[i] = goal.target_joints[i] 
        else:
            try:
                # Convert target point to target joints (find an IK solution)
                target_joints = self.transform_target_point(goal.target_point)
            except (Exception, tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("%s: Aborted: IK Transform Failure", NAME)
                self.result.success = False
                self.server.set_aborted()
                return

        # Publish goal to controllers
        self.shoulder_pan_pub.publish(target_joints[0])
        self.shoulder_tilt_pub.publish(target_joints[1])
        self.elbow_tilt_pub.publish(target_joints[2])
        self.wrist_rotate_pub.publish(target_joints[3])
        
        # Initialize loop variables
        start_time = rospy.Time.now()

        while (math.fabs(target_joints[0] - self.shoulder_pan.process_value) > self.ERROR_THRESHOLD or \
                math.fabs(target_joints[1] - self.shoulder_tilt.process_value) > self.ERROR_THRESHOLD or \
                math.fabs(target_joints[2] - self.elbow_tilt.process_value) > self.ERROR_THRESHOLD or \
                math.fabs(target_joints[3] - self.wrist_rotate.process_value) > self.ERROR_THRESHOLD):
		
	        # Cancel exe if another goal was received (i.e. preempt requested)
            if self.server.is_preempt_requested():
                rospy.loginfo("%s: Aborted: Action Preempted", NAME)
                self.result.success = False
                self.server.set_preempted()
                break

            # Publish current arm position as feedback
            self.feedback.arm_position = [self.shoulder_pan.process_value, self.shoulder_tilt.process_value, \
                    self.elbow_tilt.process_value, self.wrist_rotate.process_value]
            self.server.publish_feedback(self.feedback)
            
            # Abort if timeout
            current_time = rospy.Time.now()
            if (current_time - start_time > self.TIMEOUT_THRESHOLD):
                rospy.loginfo("%s: Aborted: Action Timeout", NAME)
                self.result.success = False
                self.server.set_aborted()
                break

            r.sleep()

        if (self.result.success):
            rospy.loginfo("%s: Goal Completed", NAME)
            self.wait_for_latest_controller_states(rospy.Duration(2.0))
            self.result.arm_position = [self.shoulder_pan.process_value, self.shoulder_tilt.process_value, \
                    self.elbow_tilt.process_value, self.wrist_rotate.process_value]
            self.server.set_succeeded(self.result)



if __name__ == '__main__':
    try:
        a = SmartArmActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

