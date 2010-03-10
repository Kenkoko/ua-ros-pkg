#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Arizona Robotics Research Group,
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
roslib.load_manifest('wubble_camera_pan_tilt_controller')

import rospy
from ax12_driver_core.ax12_const import *
from ax12_driver_core.ax12_user_commands import *
from ax12_driver_core.ax12_io import rad_to_raw, raw_to_rad
from ax12_driver_core.msg import MotorStateList
from ua_controller_msgs.msg import JointState
from ua_controller_msgs.msg import JointStateList
from std_msgs.msg import Float64
from threading import Thread

class DriverControl:
    def __init__(self, out_cb):
        self.camera_pan_tilt = CameraPanTiltAX12(out_cb)
        
    def initialize(self):
        return self.camera_pan_tilt.initialize()
        
    def start(self):
        self.camera_pan_tilt.start()
        
    def stop(self):
        self.camera_pan_tilt.stop()

class CameraPanTiltAX12():
    def __init__(self, out_cb):
        self.send_packet_callback = out_cb
        self.running = False

        pan_topic = rospy.get_param('head_controller/pan/topic', 'head_pan_controller')
        tilt_topic = rospy.get_param('head_controller/tilt/topic', 'head_tilt_controller')

        self.pan_motor_id = rospy.get_param('head_controller/pan/motor/id', 6)
        self.tilt_motor_id = rospy.get_param('head_controller/tilt/motor/id', 5)
        self.pan_joint_name = rospy.get_param('head_controller/pan/joint_name', 'head_pan_joint')
        self.tilt_joint_name = rospy.get_param('head_controller/tilt/joint_name', 'head_tilt_joint')
        
        # Pan limits
        self.pan_initial_position_raw = rospy.get_param('head_controller/pan/motor/init', 666) # scary
        self.pan_min_angle_raw = rospy.get_param('head_controller/pan/motor/min', 359)
        self.pan_max_angle_raw = rospy.get_param('head_controller/pan/motor/max', 973)
        
        self.pan_min_angle = (self.pan_min_angle_raw - self.pan_initial_position_raw) * AX_RAD_RAW_RATIO  # looking to the right from robot's perspective
        self.pan_max_angle = (self.pan_max_angle_raw - self.pan_initial_position_raw) * AX_RAD_RAW_RATIO  # looking to the left from robot's perspective
        
        # Tilt limits
        self.tilt_initial_position_raw = rospy.get_param('head_controller/tilt/motor/init', 514)
        self.tilt_min_angle_raw = rospy.get_param('head_controller/tilt/motor/min', 275)
        self.tilt_max_angle_raw = rospy.get_param('head_controller/tilt/motor/max', 634)
        
        self.tilt_min_angle = (self.tilt_min_angle_raw - self.tilt_initial_position_raw) * AX_RAD_RAW_RATIO  # looking down from robot's perspective
        self.tilt_max_angle = (self.tilt_max_angle_raw - self.tilt_initial_position_raw) * AX_RAD_RAW_RATIO  # looking up from robot's perspective

        # Get raw state from actual hardware motors
        self.motor_states_sub = rospy.Subscriber('motor_states', MotorStateList, self.process_motor_states)
        
        self.pan_state_pub = rospy.Publisher(pan_topic + '/state', JointStateList)
        self.pan_command_sub = rospy.Subscriber(pan_topic + '/command', Float64, self.do_pan)

        self.tilt_state_pub = rospy.Publisher(tilt_topic + '/state', JointStateList)
        self.tilt_command_sub = rospy.Subscriber(tilt_topic + '/command', Float64, self.do_tilt)

    def initialize(self):
        # verify that the expected motors are connected and responding
        available_ids = rospy.get_param('ax12/connected_ids', [])
        if not self.pan_motor_id in available_ids or not self.tilt_motor_id in available_ids:
            rospy.logwarn("One or more of the specified motor ids are not connected and responding.")
            rospy.logwarn("Available ids: %s" %str(available_ids))
            rospy.logwarn("Camera Pan/Tilt ids: %d, %d" %(self.pan_motor_id, self.tilt_motor_id))
            return False
        
        pan_mcv = (self.pan_motor_id, 100)  # mcv = "Move Command Value"
        tilt_mcv = (self.tilt_motor_id, 100)
        self.send_packet_callback((AX_GOAL_SPEED, [pan_mcv, tilt_mcv]))
        return True

    def start(self):
        self.running = True
        
    def stop(self):
        self.running = False
        self.pan_state_pub.unregister()
        self.tilt_state_pub.unregister()
        self.pan_command_sub.unregister()
        self.tilt_command_sub.unregister()
        
    def __pan_angle_to_raw_position(self, angle):
        if angle < self.pan_min_angle: angle = self.pan_min_angle
        elif angle > self.pan_max_angle: angle = self.pan_max_angle
        return rad_to_raw(self.pan_initial_position_raw, self.pan_min_angle_raw, self.pan_max_angle_raw, angle)
            
    def __tilt_angle_to_raw_position(self, angle):
        if angle < self.tilt_min_angle: angle = self.tilt_min_angle
        elif angle > self.tilt_max_angle: angle = self.tilt_max_angle
        return rad_to_raw(self.tilt_initial_position_raw, self.tilt_min_angle_raw, self.tilt_max_angle_raw, angle)
            
    def process_motor_states(self, state_list):
        if self.running:
            state = filter(lambda state: state.id == self.pan_motor_id or state.id == self.tilt_motor_id, state_list.motor_states)
            if len(state) == 2:
                camera_state = state[0]
                if camera_state.id == self.pan_motor_id:
                    pan_js = JointState(self.pan_joint_name,
                                        raw_to_rad(self.pan_initial_position_raw, camera_state.position),
                                        (camera_state.speed / AX_TICKS) * AX_MAX_SPEED_RAD,
                                        [self.pan_motor_id],
                                        camera_state.moving)
                else:
                    tilt_js = JointState(self.tilt_joint_name,
                                         raw_to_rad(self.tilt_initial_position_raw, camera_state.position),
                                         (camera_state.speed / AX_TICKS) * AX_MAX_SPEED_RAD,
                                         [self.tilt_motor_id],
                                         camera_state.moving)
                                         
                camera_state = state[1]
                if camera_state.id == self.pan_motor_id:
                    pan_js = JointState(self.pan_joint_name,
                                        raw_to_rad(self.pan_initial_position_raw, camera_state.position),
                                         (camera_state.speed / AX_TICKS) * AX_MAX_SPEED_RAD,
                                        [self.pan_motor_id],
                                        camera_state.moving)
                else:
                    tilt_js = JointState(self.tilt_joint_name,
                                         raw_to_rad(self.tilt_initial_position_raw, camera_state.position),
                                         (camera_state.speed / AX_TICKS) * AX_MAX_SPEED_RAD,
                                         [self.tilt_motor_id],
                                         camera_state.moving)
                                         
                self.pan_state_pub.publish([pan_js])
                self.tilt_state_pub.publish([tilt_js])
                
    def do_pan(self, pan):
        pan_mcv = (self.pan_motor_id, self.__pan_angle_to_raw_position(pan.data))
        self.send_packet_callback((AX_GOAL_POSITION, [pan_mcv]))

    def do_tilt(self, tilt):
        tilt_mcv = (self.tilt_motor_id, self.__tilt_angle_to_raw_position(tilt.data))
        self.send_packet_callback((AX_GOAL_POSITION, [tilt_mcv]))

