#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Arizona Robotics Research Group,
# University of Arizona. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Antons Rebguns
#

import roslib
roslib.load_manifest('wubble2_robot')

import rospy
from pr2_controllers_msgs.msg import JointControllerState
from dynamixel_hardware_interface.msg import JointState

class DynamixelToPR2StateMsgs:
    def __init__(self):
        rospy.init_node('dynamixel_to_pr2_state_msgs', anonymous=True)
        
        self.robot_controllers = ('shoulder_pitch_controller',
                                  'shoulder_pan_controller',
                                  'upperarm_roll_controller',
                                  'elbow_flex_controller',
                                  'forearm_roll_controller',
                                  'wrist_pitch_controller',
                                  'wrist_roll_controller',
                                  'left_finger_controller',
                                  'right_finger_controller',
                                  'head_pan_controller',
                                  'head_tilt_controller',
                                  'neck_tilt_controller')
                                  
        self.joint_to_publisher_state = {}
        
        for controller in self.robot_controllers:
            joint_name = rospy.get_param(controller + '/joint')
            publisher = rospy.Publisher(controller + '/state_pr2_msgs', JointControllerState)
            state = JointControllerState()
            self.joint_to_publisher_state[joint_name] = {'publisher': publisher, 'state': state}
            
        [rospy.Subscriber(controller + '/state', JointState, self.handle_controller_state) for controller in self.robot_controllers]
        [rospy.wait_for_message(controller + '/state', JointState)]
        
    def handle_controller_state(self, msg):
        state = self.joint_to_publisher_state[msg.name]['state']
        state.set_point = msg.target_position
        state.process_value = msg.position
        state.process_value_dot = msg.velocity
        state.error = msg.position - msg.target_position
        state.command = msg.load
        state.header.stamp = msg.header.stamp
        self.joint_to_publisher_state[msg.name]['publisher'].publish(state)

if __name__ == '__main__':
    try:
        translator = DynamixelToPR2StateMsgs()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

