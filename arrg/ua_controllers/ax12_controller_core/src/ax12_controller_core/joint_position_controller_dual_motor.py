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

class JointPositionControllerDualAX12(JointControllerAX12):
    def __init__(self, out_cb, param_path):
        JointControllerAX12.__init__(self, out_cb, param_path)
        
        self.master_id = rospy.get_param(self.topic_name + '/motor_master/id')
        self.master_initial_position_raw = rospy.get_param(self.topic_name + '/motor_master/init')
        self.master_min_angle_raw = rospy.get_param(self.topic_name + '/motor_master/min')
        self.master_max_angle_raw = rospy.get_param(self.topic_name + '/motor_master/max')
        
        self.slave_id = rospy.get_param(self.topic_name + '/motor_slave/id')
        
        self.flipped = self.master_min_angle_raw > self.master_max_angle_raw
        
        if self.flipped:
            self.master_min_angle = (self.master_initial_position_raw - self.master_min_angle_raw) * AX_RAD_RAW_RATIO
            self.master_max_angle = (self.master_initial_position_raw - self.master_max_angle_raw) * AX_RAD_RAW_RATIO
        else:
            self.master_min_angle = (self.master_min_angle_raw - self.master_initial_position_raw) * AX_RAD_RAW_RATIO
            self.master_max_angle = (self.master_max_angle_raw - self.master_initial_position_raw) * AX_RAD_RAW_RATIO
            
    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('ax12/connected_ids', [])
        if not (self.master_id in available_ids and
                self.slave_id in available_ids):
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified ids: %d %d' % (self.master_id, self.slave_id))
            return False
            
        self.set_speed(self.joint_speed)
        return True
        
    def set_speed(self, speed):
        if speed < 0: speed = 0
        elif speed > AX_MAX_SPEED_RAD: speed = AX_MAX_SPEED_RAD
        speed_raw = int(round((speed / AX_MAX_SPEED_RAD) * AX_MAX_POSITION))
        mcv_master = (self.master_id, speed_raw if speed_raw > 0 else 1)
        mcv_slave = (self.slave_id, mcv_master[1])
        self.send_packet_callback((AX_GOAL_SPEED, [mcv_master, mcv_slave]))
        
    def process_set_speed(self, req):
        self.set_speed(req.speed)
        return []
        
    def process_torque_enable(self, req):
        mcv_master = (self.master_id, req.torque_enable)
        mcv_slave = (self.slave_id, req.torque_enable)
        self.send_packet_callback((AX_TORQUE_ENABLE, [mcv_master, mcv_slave]))
        return []
        
    def process_motor_states(self, state_list):
        if self.running:
            state = filter(lambda state: state.id == self.master_id, state_list.motor_states)
            if state:
                state = state[0]
                joint_state = JointState(name=self.joint_name,
                                         motor_ids=[self.master_id],
                                         goal_pos=self.raw_to_rad(state.goal, self.master_initial_position_raw, self.flipped),
                                         current_pos=self.raw_to_rad(state.position, self.master_initial_position_raw, self.flipped),
                                         error=state.error * AX_RAD_RAW_RATIO,
                                         velocity=(state.speed / AX_TICKS) * AX_MAX_SPEED_RAD,
                                         load=state.load,
                                         is_moving=state.moving)
                joint_state.header.stamp = state_list.header.stamp
                self.joint_state_pub.publish(joint_state)
                
    def process_command(self, msg):
        angle = msg.data
        if angle < self.master_min_angle: angle = self.master_min_angle
        elif angle > self.master_max_angle: angle = self.master_max_angle
        mcv_master = (self.master_id, self.rad_to_raw(angle, self.master_initial_position_raw, self.flipped))
        mcv_slave = (self.slave_id, AX_MAX_POSITION - mcv_master[1])
        self.send_packet_callback((AX_GOAL_POSITION, [mcv_master, mcv_slave]))

