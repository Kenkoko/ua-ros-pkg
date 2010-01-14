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
from core import commands
from msg import JointState
from msg import JointStateList
from threading import Thread, Event
from std_msgs.msg import Int32

class DriverControl:
    def __init__(self, in_cb, out_cb):
        self.laser_tilt = LaserTiltAX12(in_cb, out_cb)

    def start(self):
        try:
            self.laser_tilt.start()
            rospy.spin()
        except rospy.ROSInterruptException: pass
        
    def stop(self):
        self.laser_tilt.stop()

class LaserTiltAX12():
    def __init__(self, in_cb, out_cb):
        self.send_packet_callback = out_cb
        self.get_motor_state = in_cb
                
        self.motor_state_pub = rospy.Publisher('ax12/laser_tilt/state', JointStateList)
        #rospy.init_node('laser_tilt_driver', anonymous=False)
        rospy.Subscriber('laser_tilt/cycles', Int32, self.process_new_cycles)

        self.motor_id = rospy.get_param('~motor_id', 9)
        self.step_num = rospy.get_param('~step_num', 5)
        self.update_rate = rospy.get_param("~update_rate", 5)
        self.num_cycles = 0
        
        # wait for publisher/init_node to finish initializing
        rospy.sleep(1)
        mcv = (self.motor_id, 0)
        self.send_packet_callback((commands.AX_GOAL_SPEED, [mcv]))

        self.initial_position_raw = 817
        self.min_angle_raw = 817
        self.max_angle_raw = 663

        self.initial_position_angle = 0
        self.min_angle = 0.0
        self.max_angle = 45.0

        self.__full_range_raw = abs(self.max_angle_raw - self.min_angle_raw)
        self.__full_range_deg = abs(self.max_angle - self.min_angle)
        self.__single_step_angle = self.__full_range_deg / self.step_num
	    
        self.event = Event()
        self.worker = Thread(target=self.control)
        self.worker2 = Thread(target=self.process_motor_states)
                
    def start(self):
        self.worker.start()
        self.worker2.start()

    def stop(self):
        rospy.signal_shutdown('Stopping laser_tilt_ax12 driver')
        self.event.set()

    def __angle_to_raw_position(self, angle):
        if angle < self.min_angle: angle = self.min_angle
        elif angle > self.max_angle: angle = self.max_angle
        rel_ang = angle / self.__full_range_deg
        rel_raw = self.__full_range_raw * rel_ang
        return int(self.initial_position_raw - rel_raw)

    def process_motor_states(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            state_dict_list = self.get_motor_state([self.motor_id])
            if state_dict_list:
                self.state = state_dict_list[0]
                self.motor_state_pub.publish([JointState(self.state['id'], self.state['position'], 0,0,0)])
            rate.sleep()
    
    def process_new_cycles(self, data):
        self.num_cycles = data.data
        self.event.set()
    
    def control(self):
        while not rospy.is_shutdown():
            self.event.wait()
            while not self.num_cycles == 0:
                for i in range(self.step_num):
                    angle = (i + 1) *  self.__single_step_angle + self.initial_position_angle
                    mcv = (self.motor_id, self.__angle_to_raw_position(angle))
                    self.send_packet_callback((commands.AX_GOAL_POSITION, [mcv]))
                    rospy.sleep(0.75 / (self.step_num - 1))
                mcv = (self.motor_id, self.initial_position_raw)
                self.send_packet_callback((commands.AX_GOAL_POSITION, [mcv]))
                if self.num_cycles > 0:
                    self.num_cycles -= 1
                rospy.sleep(0.25)
            self.event.clear()

