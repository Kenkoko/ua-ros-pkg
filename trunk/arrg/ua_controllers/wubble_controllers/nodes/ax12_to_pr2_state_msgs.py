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
roslib.load_manifest('wubble_controllers')

import rospy
from pr2_controllers_msgs import JointControllerState
from ua_controller_msgs.msg import JointState

class AX12ToPR2StateMsgs:
    def __init__(self, out_cb, param_path):
        arm_controllers = {'shoulder_pan_controller': 'shoulder_pan_controller',
                           'shoulder_tilt_controller': 'shoulder_tilt_controller',
                           'elbow_tilt_controller': 'elbow_tilt_controller',
                           'wrist_rotate_controller': 'wrist_rotate_controller',
                           'finger_right_controller': 'finger_right_controller',
                           'finger_left_controller': 'finger_left_controller'}
                           
        head_controllers = {'head_pan_controller': 'head_pan_controller',
                            'head_tilt_controller': 'head_tilt_controller'}
        laser_controllers = {'laser_tilt_controller': 'laser_tilt_controller'}
        
        self.arm_state_pub = [rospy.Publisher(c + '/state_pr2_msgs', JointControllerState) for c in arm_controllers]
        self.head_state_pub = [rospy.Publisher(c + '/state_pr2_msgs', JointControllerState) for c in head_controllers]
        self.laser_state_pub = [rospy.Publisher(c + '/state_pr2_msgs', JointControllerState) for c in laser_controllers]
        
        rospy.init_node('ax12_to_pr2_state_msgs', anonymous=True)
        
        self.arm_state_sub = [rospy.Subscriber(c + '/state', JointState, self.handle_arm_state) for c in arm_controllers]
        self.head_state_sub = [rospy.Subscriber(c + '/state', JointState, self.handle_head_state) for c in head_controllers]
        self.laser_state_sub = [rospy.Subscriber(c + '/state', JointState, self.handle_laser_state) for c in laser_controllers]
        
    def handle_arm_state(self, msg):
        jcs = JointControllerState(set_point=msg.goal,
                                   process_value=msg.angle,
                                   process_value_dot=msg.speed,
                                   error=msg.error)
        self.arm_state_pub[msg.name].publish(jcs)
        
    def handle_head_state(self, msg):
        jcs = JointControllerState(set_point=msg.goal,
                                   process_value=msg.angle,
                                   process_value_dot=msg.speed,
                                   error=msg.error)
        self.head_state_pub[msg.name].publish(jcs)
        
    def handle_laser_state(self, msg):
        jcs = JointControllerState(set_point=msg.goal,
                                   process_value=msg.angle,
                                   process_value_dot=msg.speed,
                                   error=msg.error)
        self.laser_state_pub[msg.name].publish(jcs)

if __name__ == '__main__':
    try:
        translator = AX12ToPR2StateMsgs()
    except rospy.ROSInterruptException:
        pass

