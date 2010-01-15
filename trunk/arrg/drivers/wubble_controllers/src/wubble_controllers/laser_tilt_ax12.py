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
from threading import Thread, Event
from std_msgs.msg import Int32

class DriverControl:
    def __init__(self, in_cb, out_cb):
        self.laser_tilt = LaserTiltAX12(in_cb, out_cb)

    def start(self):
        self.laser_tilt.start()
        
    def stop(self):
        self.laser_tilt.stop()

class LaserTiltAX12():
    def __init__(self, in_cb, out_cb):
        self.send_packet_callback = out_cb
        self.get_motor_state = in_cb
                
        self.motor_state_pub = rospy.Publisher('laser_tilt_controller/state', JointStateList)
        self.cycles_sub = rospy.Subscriber('laser_tilt_controller/cycles', Int32, self.process_new_cycles)

        self.motor_id = rospy.get_param('laser_tilt_controller/motor_id', 9)
        self.step_num = rospy.get_param('laser_tilt_controller/step_num', 5)
        self.update_rate = rospy.get_param('laser_tilt_controller/update_rate', 5)
        self.num_cycles = 0
        
        mcv = (self.motor_id, 0)
        self.send_packet_callback((AX_GOAL_SPEED, [mcv]))

        self.initial_position_raw = 817
        self.min_angle_raw = 817
        self.max_angle_raw = 663

        self.initial_position_angle = 0
        self.min_angle = 0.0    # laser parallel to the base
        self.max_angle = 45.0   # laser looking up at that angle

        self.__full_range_raw = abs(self.max_angle_raw - self.min_angle_raw)
        self.__full_range_deg = abs(self.max_angle - self.min_angle)
        self.__single_step_angle = self.__full_range_deg / self.step_num
	    
        self.event = Event()
        self.tilt_processor = Thread(target=self.do_tilt)
        self.state_processor = Thread(target=self.process_motor_states)
                
    def start(self):
        self.running = True
        self.tilt_processor.start()
        self.state_processor.start()

    def stop(self):
        self.running = False
        self.event.set()
        self.motor_state_pub.unregister()
        self.cycles_sub.unregister()
        
    def __angle_to_raw_position(self, angle):
        if angle < self.min_angle: angle = self.min_angle
        elif angle > self.max_angle: angle = self.max_angle
        rel_ang = angle / self.__full_range_deg
        rel_raw = self.__full_range_raw * rel_ang
        return int(round(self.initial_position_raw - rel_raw))

    def __raw_position_to_angle(self, raw):
        return int(round((self.initial_position_raw - raw) * AX_DEG_RAW_RATIO))
            
    def process_motor_states(self):
        rate = rospy.Rate(self.update_rate)
        while self.running:
            state_dict_list = self.get_motor_state([self.motor_id])
            if state_dict_list:
                state = state_dict_list[0]
                state['position'] = self.__raw_position_to_angle(state['position'])
                self.motor_state_pub.publish([JointState(**state)])
            rate.sleep()
    
    def process_new_cycles(self, data):
        self.num_cycles = data.data
        self.event.set()
    
    def do_tilt(self):
        while self.running:
            self.event.wait()
            while not self.num_cycles == 0:
                for i in range(self.step_num):
                    angle = (i + 1) *  self.__single_step_angle + self.initial_position_angle
                    mcv = (self.motor_id, self.__angle_to_raw_position(angle))
                    self.send_packet_callback((AX_GOAL_POSITION, [mcv]))
                    rospy.sleep(0.75 / (self.step_num - 1))
                mcv = (self.motor_id, self.initial_position_raw)
                self.send_packet_callback((AX_GOAL_POSITION, [mcv]))
                if self.num_cycles > 0:
                    self.num_cycles -= 1
                rospy.sleep(0.25)
            self.event.clear()

