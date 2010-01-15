#!/usr/bin/env python

# Copyright (c) 2008, Willow Garage, Inc.  All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.  * Redistributions
#     in binary form must reproduce the above copyright notice, this list of
#     conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution. # Neither the name of
#     the Willow Garage, Inc. nor the names of its contributors may be used to
#     endorse or promote products derived from this software without specific
#     prior written permission.
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
from math import pi
from threading import Thread

import rospy
from joy.msg import Joy
from std_msgs.msg import Float64

class MoveArmXbox():
    def __init__(self):
        self.is_running = True
        self.step_size = 1.0 * pi / 180.0
        self.joy_data = None
        self.prev_time = time.time()
        
        self.r_arm_gripper_open = False
        self.l_arm_cmd = [0.0, 1.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0]
        self.r_arm_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.arm_controllers = ['shoulder_pan_position_controller',
                                'shoulder_lift_position_controller',
                                'upper_arm_roll_position_controller',
                                'elbow_flex_position_controller',
                                'forearm_roll_position_controller',
                                'wrist_flex_position_controller',
                                'wrist_roll_position_controller',
                                'gripper_position_controller']
        
        self.l_arm_pubs = [rospy.Publisher('l_' + name + '/command', Float64) for name in self.arm_controllers]
        self.r_arm_pubs = [rospy.Publisher('r_' + name + '/command', Float64) for name in self.arm_controllers]
        
        rospy.init_node('move_arm_joy', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.read_joystick_data)

    def read_joystick_data(self, data):
        self.joy_data = data
        cur_time = time.time()
        timediff = cur_time - self.prev_time
        self.prev_time = cur_time
        if data.buttons[2] and timediff > 0.1: self.toggle_r_arm_gripper()

    def bound(self, number, lower, upper):
        if (number < lower): return lower
        elif (number > upper): return upper
        else: return number 

    def toggle_r_arm_gripper(self):
        if self.r_arm_gripper_open: self.r_arm_cmd[7] = 0.00
        else: self.r_arm_cmd[7] = 0.06
        self.r_arm_gripper_open = not self.r_arm_gripper_open

    def update_arm_position(self):
        while self.is_running:
            if self.joy_data:
                self.r_arm_cmd[1] += -1 * self.joy_data.axes[7] * self.step_size
                self.r_arm_cmd[3] += -1 * self.joy_data.axes[6] * self.step_size
                self.r_arm_cmd[1] = self.bound(self.r_arm_cmd[1], -0.35, 1.0)
                self.r_arm_cmd[3] = self.bound(self.r_arm_cmd[3], -2.0, 0.0)
            
            for i in range(0, len(self.r_arm_pubs)):
                self.r_arm_pubs[i].publish(self.r_arm_cmd[i])
                self.l_arm_pubs[i].publish(self.l_arm_cmd[i])
            
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        move_arm = MoveArmXbox()
        t = Thread(target=move_arm.update_arm_position)
        t.start()
        rospy.spin()
        move_arm.alive = False
        t.join()
    except rospy.ROSInterruptException: pass

