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
from msg import PanTilt
from threading import Thread

class DriverControl:
    def __init__(self, in_cb, out_cb):
        self.camera_pan_tilt = CameraPanTiltAX12(in_cb, out_cb)

    def start(self):
        self.camera_pan_tilt.start()
        
    def stop(self):
        self.camera_pan_tilt.stop()

class CameraPanTiltAX12():
    def __init__(self, in_cb, out_cb):
        self.send_packet_callback = out_cb
        self.get_motor_states = in_cb
                
        self.motor_state_pub = rospy.Publisher('camera_pan_tilt_controller/state', JointStateList)
        self.pan_tilt_sub = rospy.Subscriber('camera_pan_tilt_controller/pan_tilt', PanTilt, self.do_pan_tilt)

        self.pan_motor_id = rospy.get_param('camera_pan_tilt_controller/pan_motor_id', 6)
        self.tilt_motor_id = rospy.get_param('camera_pan_tilt_controller/tilt_motor_id', 5)
        self.update_rate = rospy.get_param("camera_pan_tilt_controller/update_rate", 30)

        pan_mcv = (self.pan_motor_id, 100)
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

        self.state_processor = Thread(target=self.process_motor_states)
                
    def start(self):
        self.running = True
        self.state_processor.start()

    def stop(self):
        self.running = False
        self.motor_state_pub.unregister()
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

    def process_motor_states(self):
        rate = rospy.Rate(self.update_rate)
        while self.running:
            state_dict_list = self.get_motor_states([self.pan_motor_id, self.tilt_motor_id])
            if state_dict_list:
                pan_state = state_dict_list[0]
                tilt_state = state_dict_list[1]
                pan_state['position'] = self.__raw_pan_to_angle(pan_state['position'])
                tilt_state['position'] = self.__raw_tilt_to_angle(pan_state['position'])
                pan_js = JointState(**pan_state)
                tilt_js = JointState(**tilt_state)
                self.motor_state_pub.publish([pan_js, tilt_js])
            rate.sleep()
    
    def do_pan_tilt(self, pan_tilt):
        pan_mcv = (self.pan_motor_id, self.__pan_angle_to_raw_position(pan_tilt.pan))
        tilt_mcv = (self.tilt_motor_id, self.__tilt_angle_to_raw_position(pan_tilt.tilt))
        self.send_packet_callback((AX_GOAL_POSITION, [pan_mcv, tilt_mcv]))

