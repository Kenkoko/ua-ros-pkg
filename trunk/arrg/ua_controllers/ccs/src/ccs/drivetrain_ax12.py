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
roslib.load_manifest('ccs')

import rospy
from ccs_const import *
from wubble_controllers.core.commands import *
from ccs.msg import DrivetrainMove

class DriverControl:
    def __init__(self, out_cb):
        self.drivetrain = DrivetrainAX12(out_cb)

    def start(self):
        self.drivetrain.start()
        
    def stop(self):
        self.drivetrain.stop()

class DrivetrainAX12():
    def __init__(self, out_cb):
        self.send_packet_callback = out_cb

        self.drivetrain_sub = rospy.Subscriber('charlie_controller/drivetrain', DrivetrainMove, self.do_drive)

        self.front_right_motor_id = rospy.get_param('charlie_controller/front_right_motor_id', 1)
        self.front_left_motor_id = rospy.get_param('charlie_controller/front_left_motor_id', 2)
        self.back_right_motor_id = rospy.get_param('charlie_controller/back_right_motor_id', 3)
        self.back_left_motor_id = rospy.get_param('charlie_controller/back_left_motor_id', 4)

    def start(self):
        self.running = True

    def stop(self):
        self.running = False
        self.drivetrain_sub.unregister()

    def do_drive(self, move):
        if self.running:
            if move.speed > 1023:
                move.speed = 1023
            elif move.speed < 0:
                move.speed = 0
            if move.direction == FORWARD_DIRECTION:
                front_right_mcv = (self.front_right_motor_id, -move.speed)
                front_left_mcv = (self.front_left_motor_id, move.speed)
                back_right_mcv = (self.back_right_motor_id, -move.speed)
                back_left_mcv = (self.back_left_motor_id, move.speed)
            elif move.direction == BACKWARD_DIRECTION:
                front_right_mcv = (self.front_right_motor_id, move.speed)
                front_left_mcv = (self.front_left_motor_id, -move.speed)
                back_right_mcv = (self.back_right_motor_id, move.speed)
                back_left_mcv = (self.back_left_motor_id, -move.speed)
            elif move.direction == LEFT_DIRECTION:
                front_right_mcv = (self.front_right_motor_id, -move.speed)
                front_left_mcv = (self.front_left_motor_id, -move.speed)
                back_right_mcv = (self.back_right_motor_id, -move.speed)
                back_left_mcv = (self.back_left_motor_id, -move.speed)
            elif move.direction == RIGHT_DIRECTION:
                front_right_mcv = (self.front_right_motor_id, move.speed)
                front_left_mcv = (self.front_left_motor_id, move.speed)
                back_right_mcv = (self.back_right_motor_id, move.speed)
                back_left_mcv = (self.back_left_motor_id, move.speed)
            elif move.direction == STOP:
                front_right_mcv = (self.front_right_motor_id, 0)
                front_left_mcv = (self.front_left_motor_id, 0)
                back_right_mcv = (self.back_right_motor_id, 0)
                back_left_mcv = (self.back_left_motor_id, 0)
            self.send_packet_callback((AX_GOAL_SPEED, [front_right_mcv, front_left_mcv, back_right_mcv, back_left_mcv]))