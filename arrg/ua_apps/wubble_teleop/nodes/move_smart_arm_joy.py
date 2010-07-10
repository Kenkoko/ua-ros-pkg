#!/usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group. All rights reserved.
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

# Author: Antons Rebguns
# Author: Daniel Hewlett

PKG = 'wubble_teleop'

import roslib; roslib.load_manifest(PKG)

import time
from threading import Thread

import rospy
from joy.msg import Joy
from std_msgs.msg import Float64

class MoveSmartArm():
    def __init__(self):
        self.is_running = True
        # twice the resolution of an AX-12 motor
        self.step_size = 2.0 * 0.005113269
        self.joy_data = None
        
        # True = controlling elbow tilt and wrist rotate joints
        # False = controlling shoulder pan and tilt joints
        self.upper_arm = True
        
        self.gripper_open = False
        self.arm_cmd = [0.0, 1.972222, -1.972222, 0.0, -0.240323643, 0.245436912]
        
        self.arm_controllers = ['shoulder_pan_controller',
                                'shoulder_tilt_controller',
                                'elbow_tilt_controller',
                                'wrist_rotate_controller',
                                'finger_left_controller',
                                'finger_right_controller']
                                
        self.arm_pubs = [rospy.Publisher(name + '/command', Float64) for name in self.arm_controllers]
        
        rospy.init_node('move_smart_arm_joy', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.read_joystick_data)

    def read_joystick_data(self, data):
        self.joy_data = data
        if data.buttons[2]: self.gripper_open = not self.gripper_open
        if data.buttons[0]: self.upper_arm = not self.upper_arm

    def bound(self, number, lower, upper):
        if (number < lower): return lower
        elif (number > upper): return upper
        else: return number

    def update_arm_position(self):
        while self.is_running:
            if self.joy_data:
                # control upper arm
                if self.upper_arm:
                    self.arm_cmd[2] += self.joy_data.axes[4] * self.step_size
                    self.arm_cmd[3] += self.joy_data.axes[3] * self.step_size
                    self.arm_cmd[2] = self.bound(self.arm_cmd[2], -2.0, 2.0)
                    self.arm_cmd[3] = self.bound(self.arm_cmd[3], -2.0, 2.0)
                # control lower arm
                else:
                    self.arm_cmd[1] += self.joy_data.axes[4] * self.step_size
                    self.arm_cmd[0] += self.joy_data.axes[3] * self.step_size
                    self.arm_cmd[1] = self.bound(self.arm_cmd[1], -2.0, 2.0)
                    self.arm_cmd[0] = self.bound(self.arm_cmd[0], -2.0, 2.0)
                    
                # control gripper
                if self.joy_data.buttons[2]:
                    if self.gripper_open:
                        self.arm_cmd[4] = -0.245436912
                        self.arm_cmd[5] = 0.245436912
                    else:
                        self.arm_cmd[4] = 0.838576116
                        self.arm_cmd[5] = -0.838576116
                        
            for i in range(0, len(self.arm_pubs)):
                self.arm_pubs[i].publish(self.arm_cmd[i])
                
            time.sleep(0.01)

if __name__ == '__main__':
    try:
        move_arm = MoveSmartArm()
        t = Thread(target=move_arm.update_arm_position)
        t.start()
        rospy.spin()
        move_arm.is_running = False
        t.join()
    except rospy.ROSInterruptException: pass

