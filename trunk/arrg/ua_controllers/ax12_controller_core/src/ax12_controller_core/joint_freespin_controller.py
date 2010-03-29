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
# Author: Cody Jorgensen
#

import roslib
roslib.load_manifest('ax12_controller_core')

import rospy
from ax12_driver_core.ax12_const import *
from ax12_driver_core.ax12_user_commands import *
from ax12_controller_core.joint_controller import JointControllerAX12
from ua_controller_msgs.msg import JointState
from std_msgs.msg import Float64

class JointFreespinControllerAX12(JointControllerAX12):
    def __init__(self, out_cb, param_path):
        JointControllerAX12.__init__(self, out_cb, param_path)
        self.motor_id = rospy.get_param(self.topic_name + '/motor/id')
        
    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('ax12/connected_ids', [])
        if not self.motor_id in available_ids:
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified id: %d' % self.motor_id)
            return False
            
        self.process_command(Float64(0.0))
        return True
        
    def set_speed(self, speed):
        if speed < -AX_MAX_SPEED_RAD: speed = -AX_MAX_SPEED_RAD
        elif speed > AX_MAX_SPEED_RAD: speed = AX_MAX_SPEED_RAD
        self.joint_speed = speed
        
    def process_set_speed(self, req):
        self.set_speed(req.speed)
        return []
        
    def process_torque_enable(self, req):
        mcv = (self.motor_id, req.torque_enable)
        self.send_packet_callback((AX_TORQUE_ENABLE, [mcv]))
        return []
        
    def process_motor_states(self, state_list):
        if self.running:
            state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
            if state:
                state = state[0]
                joint_state = JointState(name=self.joint_name,
                                         motor_ids=[self.motor_id],
                                         goal=0.0,
                                         angle=0.0,
                                         error=0.0,
                                         speed=(state.speed / AX_TICKS) * AX_MAX_SPEED_RAD,
                                         moving=state.moving)
                self.joint_state_pub.publish(joint_state)
                
    def process_command(self, msg):
        speed = msg.data
        if speed < -self.joint_speed: speed = -self.joint_speed
        elif speed > self.joint_speed: speed = self.joint_speed
        speed_raw = int(round((speed / AX_MAX_SPEED_RAD) * AX_MAX_POSITION))
        mcv = (self.motor_id, speed_raw)
        self.send_packet_callback((AX_GOAL_SPEED, [mcv]))

