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
from ccs.msg import ArmMove

class DriverControl:
    def __init__(self, out_cb):
        self.clawarm = ClawArmAX12(out_cb)
    
    def start(self):
        self.clawarm.start()
    
    def stop(self):
        self.clawarm.stop()
    
class ClawArmAX12():
    def __init__(self, out_cb):
        self.send_packet_callback = out_cb
        
        self.clawarm_sub = rospy.Subscriber('charlie_controller/clawarm', ArmMove, self.do_arm_move)
        
        shoulder_pan_motor_id = rospy.get_param('charlie_controller/%s/motor_id' %CLAW_SHOULDER_PAN, 12)
        elbow_tilt_motor_id = rospy.get_param('charlie_controller/%s/motor_id' %CLAW_ELBOW_TILT, 8)
        wrist_rotate_motor_id = rospy.get_param('charlie_controller/%s/motor_id' %CLAW_WRIST_ROTATE, 17)
        finger_right_motor_id = rospy.get_param('charlie_controller/%s/motor_id' %CLAW_FINGER_RIGHT, 7)
        finger_left_motor_id = rospy.get_param('charlie_controller/%s/motor_id' %CLAW_FINGER_LEFT, 6)
        
        self.name_to_ids = {
            CLAW_SHOULDER_PAN : shoulder_pan_motor_id,
            CLAW_ELBOW_TILT   : elbow_tilt_motor_id,
            CLAW_WRIST_ROTATE : wrist_rotate_motor_id,
            CLAW_FINGER_RIGHT : finger_right_motor_id,
            CLAW_FINGER_LEFT  : finger_left_motor_id
        }
        
        self.name_to_speed = {
            CLAW_SHOULDER_PAN : 0,
            CLAW_ELBOW_TILT   : 0,
            CLAW_WRIST_ROTATE : 0,
            CLAW_FINGER_RIGHT : 0,
            CLAW_FINGER_LEFT  : 0
        }

    def start(self):
        self.running = True
    
    def stop(self):
        self.running = False
        self.clawarm_sub.unregister()
    
    def do_arm_move(self, move):
        # Move has joint_name, motor_position, motor_speed
        if self.running:
            if not move.motor_speed == self.name_to_speed[move.joint_name]:
                # FYI: mcv stands for "motor command value"
                motor_speed_mcv = (self.name_to_ids[move.joint_name], move.motor_speed)
                self.send_packet_callback((AX_GOAL_SPEED, [motor_speed_mcv]))
            motor_position_mcv = (self.name_to_ids[move.joint_name], move.motor_position)
            self.send_packet_callback((AX_GOAL_POSITION, [motor_position_mcv]))