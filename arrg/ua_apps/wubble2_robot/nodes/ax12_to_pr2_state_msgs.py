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
from pr2_controllers_msgs.msg import JointControllerState
from ua_controller_msgs.msg import JointState

class AX12ToPR2StateMsgs:
    def __init__(self):
        self.arm_controllers = {'shoulder_pitch_controller': None,
                                'shoulder_yaw_controller': None,
                                'shoulder_roll_controller': None,
                                'elbow_pitch_controller': None,
                                'wrist_roll_controller': None,
                                'wrist_pitch_controller': None,
                                'wrist_yaw_controller': None,
                                'left_finger_controller': None,
                                'right_finger_controller': None}
                           
        self.head_controllers = {'head_pan_controller': None,
                                 'head_tilt_controller': None}
        self.laser_controllers = {'neck_tilt_controller': None}
                             
        for pub in self.arm_controllers:
            self.arm_controllers[pub] = rospy.Publisher(pub + '/state_pr2_msgs', JointControllerState)
            
        for pub in self.head_controllers:
            self.head_controllers[pub] = rospy.Publisher(pub + '/state_pr2_msgs', JointControllerState)
            
        for pub in self.laser_controllers:
            self.laser_controllers[pub] = rospy.Publisher(pub + '/state_pr2_msgs', JointControllerState)
        
        rospy.init_node('ax12_to_pr2_state_msgs', anonymous=True)
        
        [rospy.Subscriber(c + '/state', JointState, self.handle_arm_state) for c in self.arm_controllers]
        [rospy.Subscriber(c + '/state', JointState, self.handle_head_state) for c in self.head_controllers]
        [rospy.Subscriber(c + '/state', JointState, self.handle_laser_state) for c in self.laser_controllers]
        
    def handle_arm_state(self, msg):
        jcs = JointControllerState(set_point=msg.goal_pos,
                                   process_value=msg.current_pos,
                                   process_value_dot=msg.velocity,
                                   error=msg.error,
                                   command=msg.load)
        jcs.header.stamp = msg.header.stamp
        name = msg.name.replace('_joint', '_controller', 1)
        self.arm_controllers[name].publish(jcs)
        
    def handle_head_state(self, msg):
        jcs = JointControllerState(set_point=msg.goal_pos,
                                   process_value=msg.current_pos,
                                   process_value_dot=msg.velocity,
                                   error=msg.error,
                                   command=msg.load)
        jcs.header.stamp = msg.header.stamp
        name = msg.name.replace('_joint', '_controller', 1)
        self.head_controllers[name].publish(jcs)
        
    def handle_laser_state(self, msg):
        jcs = JointControllerState(set_point=msg.goal_pos,
                                   process_value=msg.current_pos,
                                   process_value_dot=msg.velocity,
                                   error=msg.error,
                                   command=msg.load)
        jcs.header.stamp = msg.header.stamp
        name = 'neck_tilt_controller'
        self.laser_controllers[name].publish(jcs)

if __name__ == '__main__':
    try:
        translator = AX12ToPR2StateMsgs()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

