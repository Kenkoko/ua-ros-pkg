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
# Author: Cara Slutter
#

from __future__ import division

import roslib
roslib.load_manifest('ax12_controller_core')

import rospy
from ax12_driver_core.ax12_const import *
from ax12_driver_core.ax12_user_commands import *
from ax12_controller_core.joint_controller import JointControllerAX12
from ua_controller_msgs.msg import JointState

class JointPositionControllerAX12(JointControllerAX12):
    def __init__(self, out_cb, param_path, port_name):
        JointControllerAX12.__init__(self, out_cb, param_path, port_name)
        
        self.motor_id = rospy.get_param(self.topic_name + '/motor/id')
        self.initial_position_raw = rospy.get_param(self.topic_name + '/motor/init')
        self.min_angle_raw = rospy.get_param(self.topic_name + '/motor/min')
        self.max_angle_raw = rospy.get_param(self.topic_name + '/motor/max')
        
        self.flipped = self.min_angle_raw > self.max_angle_raw
        
        self.joint_state = JointState(name=self.joint_name, motor_ids=[self.motor_id])

    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('dynamixel/%s/connected_ids' % self.port_namespace, [])
        if not self.motor_id in available_ids:
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified id: %d' % self.motor_id)
            return False
            
        self.radians_per_encoder_tick = rospy.get_param('dynamixel/%s/%d/radians_per_encoder_tick' % (self.port_namespace, self.motor_id))
        self.encoder_ticks_per_radian = rospy.get_param('dynamixel/%s/%d/encoder_ticks_per_radian' % (self.port_namespace, self.motor_id))
        
        if self.flipped:
            self.min_angle = (self.initial_position_raw - self.min_angle_raw) * self.radians_per_encoder_tick
            self.max_angle = (self.initial_position_raw - self.max_angle_raw) * self.radians_per_encoder_tick
        else:
            self.min_angle = (self.min_angle_raw - self.initial_position_raw) * self.radians_per_encoder_tick
            self.max_angle = (self.max_angle_raw - self.initial_position_raw) * self.radians_per_encoder_tick
            
        self.encoder_resolution = rospy.get_param('dynamixel/%s/%d/encoder_resolution' % (self.port_namespace, self.motor_id))
        self.max_position = self.encoder_resolution - 1
        self.set_speed(self.joint_speed)
        
        if self.compliance_slope is not None: self.set_compliance_slope(self.compliance_slope)
        if self.compliance_margin is not None: self.set_compliance_margin(self.compliance_margin)
        if self.compliance_punch is not None: self.set_compliance_punch(self.compliance_punch)
        if self.torque_limit is not None: self.set_torque_limit(self.torque_limit)
        return True

    def set_torque_enable(self, torque_enable):
        mcv = (self.motor_id, torque_enable)
        self.send_packet_callback((DMXL_SET_TORQUE_ENABLE, [mcv]))

    def set_speed(self, speed):
        if speed < DMXL_MIN_SPEED_RAD: speed = DMXL_MIN_SPEED_RAD
        elif speed > self.joint_max_speed: speed = self.joint_max_speed
        speed_raw = int(round(speed / DMXL_SPEED_RAD_SEC_PER_TICK))
        mcv = (self.motor_id, speed_raw)
        self.send_packet_callback((DMXL_SET_GOAL_SPEED, [mcv]))

    def set_compliance_slope(self, slope):
        if slope < DMXL_MIN_COMPLIANCE_SLOPE: slope = DMXL_MIN_COMPLIANCE_SLOPE
        elif slope > DMXL_MAX_COMPLIANCE_SLOPE: slope = DMXL_MAX_COMPLIANCE_SLOPE
        slope2 = (slope << 8) + slope
        mcv = (self.motor_id, slope2)
        self.send_packet_callback((DMXL_SET_COMPLIANCE_SLOPES, [mcv]))

    def set_compliance_margin(self, margin):
        if margin < DMXL_MIN_COMPLIANCE_MARGIN: margin = DMXL_MIN_COMPLIANCE_MARGIN
        elif margin > DMXL_MAX_COMPLIANCE_MARGIN: margin = DMXL_MAX_COMPLIANCE_MARGIN
        else: margin = int(margin)
        margin2 = (margin << 8) + margin    # pack margin_cw and margin_ccw into 2 bytes
        mcv = (self.motor_id, margin2)
        self.send_packet_callback((DMXL_SET_COMPLIANCE_MARGINS, [mcv]))

    def set_compliance_punch(self, punch):
        if punch < DMXL_MIN_PUNCH: punch = DMXL_MIN_PUNCH
        elif punch > DMXL_MAX_PUNCH: punch = DMXL_MAX_PUNCH
        else: punch = int(punch)
        mcv = (self.motor_id, punch)
        self.send_packet_callback((DMXL_SET_PUNCH, [mcv]))

    def set_torque_limit(self, max_torque):
        if max_torque > 1: max_torque = 1.0         # use all torque motor can provide
        elif max_torque < 0: max_torque = 0.0       # turn off motor torque
        raw_torque_val = int(DMXL_MAX_TORQUE_TICK * max_torque)
        mcv = (self.motor_id, raw_torque_val)
        self.send_packet_callback((DMXL_SET_TORQUE_LIMIT, [mcv]))

    def process_motor_states(self, state_list):
        if self.running:
            state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
            if state:
                state = state[0]
                self.joint_state.motor_temps = [state.temperature]
                self.joint_state.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.radians_per_encoder_tick)
                self.joint_state.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.radians_per_encoder_tick)
                self.joint_state.error = state.error * self.radians_per_encoder_tick
                self.joint_state.velocity = (state.speed / DMXL_MAX_SPEED_TICK) * DMXL_MAX_SPEED_RAD
                self.joint_state.load = state.load
                self.joint_state.is_moving = state.moving
                self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                
                self.joint_state_pub.publish(self.joint_state)

    def process_command(self, msg):
        angle = msg.data
        if angle < self.min_angle: angle = self.min_angle
        elif angle > self.max_angle: angle = self.max_angle
        mcv = (self.motor_id, self.rad_to_raw(angle, self.initial_position_raw, self.flipped, self.encoder_ticks_per_radian))
        self.send_packet_callback((DMXL_SET_GOAL_POSITION, [mcv]))

