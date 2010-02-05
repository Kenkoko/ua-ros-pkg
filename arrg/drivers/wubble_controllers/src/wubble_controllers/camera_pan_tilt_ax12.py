#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Arizona Robotics Research Group,
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

import roslib
roslib.load_manifest('wubble_controllers')

import rospy
from core.ax12_const import *
from core.commands import *
from msg import JointState
from msg import JointStateList
from msg import MotorStateList
from msg import PanTilt
from threading import Thread

class DriverControl:
    def __init__(self, out_cb):
        self.camera_pan_tilt = CameraPanTiltAX12(out_cb)

    def start(self):
        self.camera_pan_tilt.start()
        
    def stop(self):
        self.camera_pan_tilt.stop()

class CameraPanTiltAX12():
    def __init__(self, out_cb):
        self.send_packet_callback = out_cb
                
        self.joint_state_pub = rospy.Publisher('camera_pan_tilt_controller/state', JointStateList)
        self.pan_tilt_sub = rospy.Subscriber('camera_pan_tilt_controller/pan_tilt', PanTilt, self.do_pan_tilt)
        self.motor_states_sub = rospy.Subscriber('motor_states', MotorStateList, self.process_motor_states)

        self.pan_motor_id = rospy.get_param('camera_pan_tilt_controller/pan_motor_id', 6)
        self.tilt_motor_id = rospy.get_param('camera_pan_tilt_controller/tilt_motor_id', 5)
        self.pan_joint_name = rospy.get_param('camera_pan_tilt_controller/pan_joint_name', 'camera_pan_joint')
        self.tilt_joint_name = rospy.get_param('camera_pan_tilt_controller/tilt_joint_name', 'camera_tilt_joint')

        pan_mcv = (self.pan_motor_id, 100)	# mcv = "Move Command Value"
        tilt_mcv = (self.tilt_motor_id, 100)
        self.send_packet_callback((AX_GOAL_SPEED, [pan_mcv, tilt_mcv]))

        # Pan limits
        self.pan_initial_position_raw = 666 # scary
        self.pan_min_angle_raw = 973
        self.pan_max_angle_raw = 359

        self.pan_initial_position_angle = 0.0
        self.pan_min_angle = -90.0  # looking to the left from robot's perspective
        self.pan_max_angle = 90.0   # looking to the right from robot's perspective
	    
        # Tilt limits
        self.tilt_initial_position_raw = 514
        self.tilt_min_angle_raw = 239
        self.tilt_max_angle_raw = 634

        self.tilt_initial_position_angle = 0.0
        self.tilt_min_angle = -70.0  # looking down from robot's perspective
        self.tilt_max_angle = 35.0   # looking up from robot's perspective
                
    def start(self):
        self.running = True

    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.pan_tilt_sub.unregister()

    def __pan_angle_to_raw_position(self, angle):
        if angle < self.pan_min_angle: angle = self.pan_min_angle
        elif angle > self.pan_max_angle: angle = self.pan_max_angle
        angle_raw = abs(angle * AX_RAW_DEG_RATIO)

        if angle >= 0:
            pos_full_range = abs(self.pan_initial_position_raw - self.pan_max_angle_raw)
            rel_ang = angle_raw / pos_full_range
            rel_raw = pos_full_range * rel_ang
            return int(round(self.pan_initial_position_raw - rel_raw))
        else:
            neg_full_range = abs(self.pan_initial_position_raw - self.pan_min_angle_raw)
            rel_ang = angle_raw / neg_full_range
            rel_raw = neg_full_range * rel_ang
            return int(round(self.pan_initial_position + rel_raw))

    def __tilt_angle_to_raw_position(self, angle):
        if angle < self.tilt_min_angle: angle = self.tilt_min_angle
        elif angle > self.tilt_max_angle: angle = self.tilt_max_angle
        angle_raw = abs(angle * AX_RAW_DEG_RATIO)
        
        if angle >= 0:
            pos_full_range = abs(self.tilt_initial_position_raw - self.tilt_max_angle_raw)
            rel_ang = angle_raw / pos_full_range
            rel_raw = pos_full_range * rel_ang
            return int(round(self.tilt_initial_position_raw + rel_raw))
        else:
            neg_full_range =abs(self.tilt_initial_position_raw - self.tilt_min_angle_raw)
            rel_ang = angle_raw / neg_full_range
            rel_raw = neg_full_range * rel_ang
            return int(round(self.tilt_initial_position_raw - rel_raw))
                              
    def __raw_pan_to_angle(self, raw):
        if raw > self.pan_initial_position_raw:
            return -int(round((raw - self.pan_initial_position_raw) * AX_DEG_RAW_RATIO))
        else:
            return int(round((self.pan_initial_position_raw - raw) * AX_DEG_RAW_RATIO))

    def __raw_tilt_to_angle(self, raw):
        if raw < self.tilt_initial_position_raw:
            return -int(round((self.tilt_initial_position_raw - raw) * AX_DEG_RAW_RATIO))
        else:
            return int(round((raw - self.tilt_initial_position_raw) * AX_DEG_RAW_RATIO))

    def process_motor_states(self, state_list):
        if self.running:
            state = filter(lambda state: state.id == self.pan_motor_id or state.id == self.tilt_motor_id, state_list.motor_states)
            if len(state) == 2:
                camera_state = state[0]
                if camera_state.id == self.pan_motor_id:
                    pan_js = JointState(self.pan_joint_name, self.__raw_pan_to_angle(camera_state.position), [self.pan_motor_id], camera_state.moving)
                else:
                    tilt_js = JointState(self.tilt_joint_name, self.__raw_tilt_to_angle(camera_state.position), [self.tilt_motor_id], camera_state.moving)

                camera_state = state[1]
                if camera_state.id == self.pan_motor_id:
                    pan_js = JointState(self.pan_joint_name, self.__raw_pan_to_angle(camera_state.position), [self.pan_motor_id], camera_state.moving)
                else:
                    tilt_js = JointState(self.tilt_joint_name, self.__raw_tilt_to_angle(camera_state.position), [self.tilt_motor_id], camera_state.moving)

                self.joint_state_pub.publish([pan_js, tilt_js])
    
    def do_pan_tilt(self, pan_tilt):
        pan_mcv = (self.pan_motor_id, self.__pan_angle_to_raw_position(pan_tilt.pan))
        tilt_mcv = (self.tilt_motor_id, self.__tilt_angle_to_raw_position(pan_tilt.tilt))
        self.send_packet_callback((AX_GOAL_POSITION, [pan_mcv, tilt_mcv]))

