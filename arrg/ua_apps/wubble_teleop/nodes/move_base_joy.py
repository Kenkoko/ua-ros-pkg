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

# Author: Antons Rebguns

PKG = 'wubble_teleop'

import roslib; roslib.load_manifest(PKG)

import time
from math import pi
from threading import Thread

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class MoveBaseXbox():
    def __init__(self):
        self.is_running = True
        self.max_speed = 0.5                    # meters/second
        self.max_turn = 20.0 * pi / 180.0       # radians/second
        self.joy_data = None
        self.cmd_vel = Twist()
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        rospy.init_node('move_base_joy', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.read_joystick_data)

    def read_joystick_data(self, data):
        self.joy_data = data

    def stop_robot(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0

    def update_base_position(self):
        while self.is_running:
            if self.joy_data:
                if self.joy_data.buttons[0]:
                    self.stop_robot()
                else:
                    self.cmd_vel.linear.x = self.joy_data.axes[1] * self.max_speed
                    self.cmd_vel.angular.z = self.joy_data.axes[0] * self.max_turn
            
            self.cmd_vel_pub.publish(self.cmd_vel)
            
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        move_base = MoveBaseXbox()
        t = Thread(target=move_base.update_base_position)
        t.start()
        rospy.spin()
        move_base.is_running = False
        t.join()
        move_base.stop_robot()
    except rospy.ROSInterruptException: pass

