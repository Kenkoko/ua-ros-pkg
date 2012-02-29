#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Daniel Hewlett
# Author: Antons Rebguns

PKG = 'wubble_teleop'

import roslib; roslib.load_manifest(PKG)

import time
import math
from threading import Thread

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from dynamixel_hardware_interface.msg import JointState

class MoveHeadXbox():
    def __init__(self):
        self.is_running = True
        self.pan_range = [-math.pi/2, math.pi/2]
        self.tilt_range = [-2, 2]

        self.head_pan = 0.0
        self.head_tilt = 0.0

        self.actual_pan = 0.0
        self.actual_tilt = 0.0

        self.joy_data = None
       
        self.head_pan_pub = rospy.Publisher('head_pan_controller/command', Float64)
        self.head_tilt_pub = rospy.Publisher('head_tilt_controller/command', Float64)
        rospy.init_node('move_head_joy', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.read_joystick_data)
        rospy.Subscriber('head_pan_controller/state', JointState, self.read_current_pan)
        rospy.Subscriber('head_tilt_controller/state', JointState, self.read_current_tilt)

    def read_joystick_data(self, data):
        self.joy_data = data

    def bound(self, number, limits):
        if (number < limits[0]): return limits[0]
        elif (number > limits[1]): return limits[1]
        else: return number 

    def bound_pan(self, number):
        return self.bound(number, self.pan_range)

    def bound_tilt(self, number):
        return self.bound(number, self.tilt_range)

    def set_head_position(self, delta_pan, delta_tilt):
        self.head_pan = self.bound_pan(delta_pan * 0.2 + self.actual_pan)
        self.head_tilt = self.bound_tilt(delta_tilt * 0.2 + self.actual_tilt)

    def reset_head_position(self):
        self.head_pan_pub.publish(0.0)
        self.head_tilt_pub.publish(0.0)
        self.head_pan = 0.0
        self.head_tilt = 0.0

    def read_current_pan(self, data):
        self.actual_pan = data.position

    def read_current_tilt(self, data):
        self.actual_tilt = data.position

    def update_head_position(self):
        while self.is_running:
            if self.joy_data:
                if self.joy_data.buttons[4]: 
                    self.reset_head_position()
                    print "Attempting to reset head position"
                else: 
                    self.set_head_position(self.joy_data.axes[3], -self.joy_data.axes[4])
            
                #if self.joy_data.buttons[7]:
                if self.head_pan > 0.05 or self.head_pan < -0.05:
                    print 'pan:', self.head_pan
                    self.head_pan_pub.publish(self.head_pan)
                    
                if self.head_tilt > 0.05 or self.head_tilt < -0.05:
                    print 'tilt:', self.head_tilt
                    self.head_tilt_pub.publish(self.head_tilt)
            
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        move_head = MoveHeadXbox()
        t = Thread(target=move_head.update_head_position)
        t.start()
        rospy.spin()
        move_head.is_running = False
        t.join()
        move_head.reset_head_position()
    except rospy.ROSInterruptException: pass

