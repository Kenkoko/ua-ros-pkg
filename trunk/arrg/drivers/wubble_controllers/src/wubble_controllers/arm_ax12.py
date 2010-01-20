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
from msg import MotorStateList
from threading import Thread, Event
from std_msgs.msg import Int32

class DriverControl:
    def __init__(self, out_cb):
        self.arm = ArmAX12(out_cb)

    def start(self):
        self.arm.start()

    def stop(self):
        self.arm.stop()

class ArmJoint:
    def __init__(self, name, min_angle, max_angle, zero_angle_raw, min_angle_raw, max_angle_raw, master_id, inverse_id=None):
        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.zero_angle_raw = zero_angle_raw
        self.min_angle_raw = min_angle_raw
        self.max_angle_raw = max_angle_raw
        self.master_id = master_id
        self.inverse_id = inverse_id
        self.flipped = True if min_angle_raw > max_angle_raw else False

    def angle_to_raw_positions(self, angle):
        result = []
        if angle > self.max_angle: angle = self.max_angle
        elif angle < self.min_angle: angle = self.min_angle
        angle_raw = angle * AX_RAW_DEG_RATIO
        
        if self.flipped:
            result.append(self.master_id, int(round(self.zero_angle_raw - angle_raw)))
        else:
            result.append(self.master_id, int(round(self.zero_angle_raw + angle_raw)))
            
        if inverse_id: result.append(self.inverse_id, 1023 - result[0][1])
        return result

    def raw_position_to_angle(self, master_raw):
        if self.flipped:
            return int(round((self.zero_angle_raw - master_raw) * AX_DEG_RAW_RATIO))
        else:
            return int(round((master_raw - self.zero_angle_raw) * AX_DEG_RAW_RATIO))

class ArmAX12():
    def __init__(self, out_cb):
        self.send_packet_callback = out_cb
        self.running = False
        
        self.joint_state_pub = rospy.Publisher('arm_controller/state', JointStateList)
        self.motor_states_sub = rospy.Subscriber('motor_states', MotorStateList, self.process_motor_states)
        
        self.shoulder_pan_id = rospy.get_param('arm_controller/shoulder_pan/motor_id', 10)
        self.shoulder_tilt_ids = rospy.get_param('arm_controller/shoulder_tilt/motor_ids', [11, 12])
        self.elbow_tilt_ids = rospy.get_param('arm_controller/elbow_tilt/motor_ids', [13, 14])
        self.wrist_id = rospy.get_param('arm_controller/wrist/motor_id', 15)
        self.finger_right_id = rospy.get_param('arm_controller/finger_right/motor_id', 16)
        self.finger_left_id = rospy.get_param('arm_controller/finger_left/motor_id', 18)
        
        self.shoulder_pan_joint = ArmJoint('shoulder_pan_joint', -70, 70, 512, 273, 751, self.shoulder_pan_id)
        self.shoulder_tilt_joint = ArmJoint('shoulder_tilt_joint', -60, 113, 512, 717, 126, self.shoulder_tilt_ids[0], self.shoulder_tilt_ids[1])
        self.elbow_tilt_joint = ArmJoint('elbow_tilt_joint', -113, 108, 512, 126, 881, self.elbow_tilt_ids[0], self.elbow_tilt_ids[1])
        self.wrist_joint = ArmJoint('wrist_joint', -150, 150, 512, 0, 1023, self.wrist_id)
        self.finger_right_joint = ArmJoint('finger_right_joint', 0, 62, 512, 512, 724, self.finger_right_id)
        self.finger_left_joint = ArmJoint('finger_left_joint', 0, 62, 512, 512, 300, self.finger_left_id)
        
        self.ids_to_joints = {self.shoulder_pan_id: self.shoulder_pan_joint,
                              self.shoulder_tilt_ids[0]: self.shoulder_tilt_joint,
                              self.elbow_tilt_ids[0]: self.elbow_tilt_joint,
                              self.wrist_id: self.wrist_joint,
                              self.finger_right_id: self.finger_right_joint,
                              self.finger_left_id: self.finger_left_joint}
                              
        self.motor_ids = [self.shoulder_pan_id,
                          self.shoulder_tilt_ids[0], self.shoulder_tilt_ids[1], 
                          self.elbow_tilt_ids[0], self.elbow_tilt_ids[1],
                          self.wrist_id,
                          self.finger_right_id, self.finger_left_id]
                          
        initial_positions = [self.shoulder_pan_joint.zero_angle_raw,
                             self.shoulder_tilt_joint.max_angle_raw, 1023 - self.shoulder_tilt_joint.max_angle_raw,
                             self.elbow_tilt_joint.min_angle_raw, 1023 - self.elbow_tilt_joint.min_angle_raw,
                             self.wrist_joint.zero_angle_raw,
                             self.finger_right_joint.max_angle_raw, self.finger_left_joint.max_angle_raw]
                            
        mcvs = [(mid, 200) for mid in self.motor_ids]
        self.send_packet_callback((AX_GOAL_SPEED, mcvs))    # set speeds of all motors to 200
        
        mcvs = zip(self.motor_ids, initial_positions)
        self.send_packet_callback((AX_GOAL_POSITION, mcvs)) # set initial pose
         
    def start(self):
        self.running = True
        #self.tilt_processor.start()
        
    def stop(self):
        self.running = False
        #self.event.set()
        #self.joint_state_pub.unregister()
        #self.cycles_sub.unregister()
        #self.tilt_sub.unregister()
        #self.motor_states_sub.unregister()
        
    def process_motor_states(self, state_list):
        if self.running:
            states = filter(lambda state: state.id in self.motor_ids, state_list.motor_states)
            if states:
                joints = []
                for state in states:
                    if state.id in self.ids_to_joints:
                        joint = self.ids_to_joints[state.id]
                        joints.append(JointState(joint.name, joint.raw_position_to_angle(state.position), filter(None, [joint.master_id, joint.inverse_id]), state.moving))
                self.joint_state_pub.publish(joints)
                
    def do_tilt(self, angle):
        mcv = (self.motor_id, self.__angle_to_raw_position(angle.data))
        self.send_packet_callback((AX_GOAL_POSITION, [mcv]))

