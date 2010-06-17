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
roslib.load_manifest('ax12_controller_core')

import rospy
from ax12_driver_core.ax12_const import *
from ax12_driver_core.msg import MotorStateList
from ax12_controller_core.srv import SetSpeed
from ax12_controller_core.srv import TorqueEnable
from ua_controller_msgs.msg import JointState
from std_msgs.msg import Float64

import math

class JointControllerAX12:
    def __init__(self, out_cb, param_path):
        self.running = False
        self.send_packet_callback = out_cb
        self.topic_name = param_path
        self.joint_name = rospy.get_param(self.topic_name + '/joint_name')
        self.joint_speed = rospy.get_param(self.topic_name + '/joint_speed')
        
        self.speed_service = rospy.Service(self.topic_name + '/set_speed', SetSpeed, self.process_set_speed)
        self.torque_service = rospy.Service(self.topic_name + '/torque_enable', TorqueEnable, self.process_torque_enable)
        
    def initialize(self):
        raise NotImplementedError
        
    def start(self):
        self.running = True
        self.joint_state_pub = rospy.Publisher(self.topic_name + '/state', JointState)
        self.command_sub = rospy.Subscriber(self.topic_name + '/command', Float64, self.process_command)
        self.motor_states_sub = rospy.Subscriber('motor_states', MotorStateList, self.process_motor_states)
        
    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.motor_states_sub.unregister()
        self.command_sub.unregister()
        self.speed_service.shutdown('normal shutdown')
        self.torque_service.shutdown('normal shutdown')
        
    def set_speed(self, speed):
        raise NotImplementedError
        
    def process_set_speed(self, req):
        raise NotImplementedError
        
    def process_torque_enable(self, req):
        raise NotImplementedError
        
    def process_motor_states(self, state_list):
        raise NotImplementedError
        
    def process_command(self, msg):
        raise NotImplementedError
        
    def rad_to_raw(self, angle, initial_position_raw, flipped):
        """ angle is in radians """
        #print 'flipped = %s, angle_in = %f, init_raw = %d' % (str(flipped), angle, initial_position_raw)
        angle_raw = angle * AX_RAW_RAD_RATIO
        #print 'angle = %f, val = %d' % (math.degrees(angle), int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw)))
        return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))
        
    def raw_to_rad(self, raw, initial_position_raw, flipped):
        return (initial_position_raw - raw if flipped else raw - initial_position_raw) * AX_RAD_RAW_RATIO

