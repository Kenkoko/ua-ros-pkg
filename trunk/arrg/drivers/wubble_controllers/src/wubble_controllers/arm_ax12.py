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
from arm_ax12_const import *
from msg import JointState
from msg import ArmJointMove
from msg import JointStateList
from msg import MotorStateList
from std_msgs.msg import Int32

class DriverControl:
    def __init__(self, out_cb):
        self.arm = ArmAX12(out_cb)
        
    def start(self):
        self.arm.start()
        
    def stop(self):
        self.arm.stop()

class ArmJoint:
    def __init__(self, name, zero_angle_raw, min_angle_raw, max_angle_raw, master_id, inverse_id=None):
        self.name = name
        self.zero_angle_raw = zero_angle_raw
        self.min_angle_raw = min_angle_raw
        self.max_angle_raw = max_angle_raw
        self.master_id = master_id
        self.inverse_id = inverse_id
        self.flipped = True if min_angle_raw > max_angle_raw else False
        if self.flipped:
            self.min_angle = int( (zero_angle_raw - min_angle_raw) * AX_DEG_RAW_RATIO )
            self.max_angle = int( (zero_angle_raw - max_angle_raw) * AX_DEG_RAW_RATIO )
        else:
            self.min_angle = int( (min_angle_raw - zero_angle_raw) * AX_DEG_RAW_RATIO )
            self.max_angle = int( (max_angle_raw - zero_angle_raw) * AX_DEG_RAW_RATIO )
            
    def angle_to_raw_positions(self, angle):
        result = []
        if angle > self.max_angle: angle = self.max_angle
        elif angle < self.min_angle: angle = self.min_angle
        angle_raw = angle * AX_RAW_DEG_RATIO
        
        if self.flipped:
            result.append( (self.master_id, int(round(self.zero_angle_raw - angle_raw))) )  # <- tuple here
        else:
            result.append( (self.master_id, int(round(self.zero_angle_raw + angle_raw))) )  # <- tuple here
            
        if self.inverse_id: result.append( (self.inverse_id, AX_MAX_POSITION - result[0][1]) )  # <- tuple here
        return result
        
    def raw_position_to_angle(self, master_raw):
        if self.flipped:
            return int(round((self.zero_angle_raw - master_raw) * AX_DEG_RAW_RATIO))
        else:
            return int(round((master_raw - self.zero_angle_raw) * AX_DEG_RAW_RATIO))

class ArmAX12():
    def __init__(self, out_cb):
        self.send_packet_callback = out_cb
        
        shoulder_pan_id = rospy.get_param('arm_controller/%s/motor_id' % SHOULDER_PAN, 10)
        shoulder_tilt_ids = rospy.get_param('arm_controller/%s/motor_ids' % SHOULDER_TILT, [11, 12])
        elbow_tilt_ids = rospy.get_param('arm_controller/%s/motor_ids' % ELBOW_TILT, [13, 14])
        wrist_id = rospy.get_param('arm_controller/%s/motor_id' % WRIST_ROTATE, 15)
        finger_right_id = rospy.get_param('arm_controller/%s/motor_id' % FINGER_RIGHT, 16)
        finger_left_id = rospy.get_param('arm_controller/%s/motor_id' % FINGER_LEFT, 18)
        
        shoulder_pan_joint = ArmJoint(SHOULDER_PAN, 512, 751, 273, shoulder_pan_id)
        shoulder_tilt_joint = ArmJoint(SHOULDER_TILT, 512, 717, 126, shoulder_tilt_ids[0], shoulder_tilt_ids[1])
        elbow_tilt_joint = ArmJoint(ELBOW_TILT, 512, 126, 881, elbow_tilt_ids[0], elbow_tilt_ids[1])
        wrist_rotate_joint = ArmJoint(WRIST_ROTATE, 512, 0, 1023, wrist_id)
        finger_right_joint = ArmJoint(FINGER_RIGHT, 512, 512, 724, finger_right_id)
        finger_left_joint = ArmJoint(FINGER_LEFT, 512, 512, 300, finger_left_id)
        
        self.name_to_joint = {SHOULDER_PAN: shoulder_pan_joint,
                              SHOULDER_TILT: shoulder_tilt_joint,
                              ELBOW_TILT: elbow_tilt_joint,
                              WRIST_ROTATE: wrist_rotate_joint,
                              FINGER_RIGHT: finger_right_joint,
                              FINGER_LEFT: finger_left_joint}
                              
        self.ids_to_joints = {shoulder_pan_id: shoulder_pan_joint,
                              shoulder_tilt_ids[0]: shoulder_tilt_joint,
                              elbow_tilt_ids[0]: elbow_tilt_joint,
                              wrist_id: wrist_rotate_joint,
                              finger_right_id: finger_right_joint,
                              finger_left_id: finger_left_joint}
                              
        self.motor_ids = [shoulder_pan_id,
                          shoulder_tilt_ids[0], shoulder_tilt_ids[1],
                          elbow_tilt_ids[0], elbow_tilt_ids[1],
                          wrist_id,
                          finger_right_id, finger_left_id]
                          
        initial_positions = [shoulder_pan_joint.zero_angle_raw,
                             shoulder_tilt_joint.max_angle_raw, AX_MAX_POSITION - shoulder_tilt_joint.max_angle_raw,
                             elbow_tilt_joint.min_angle_raw, AX_MAX_POSITION - elbow_tilt_joint.min_angle_raw,
                             wrist_rotate_joint.zero_angle_raw,
                             finger_right_joint.max_angle_raw, finger_left_joint.max_angle_raw]
                            
        mcvs = [(mid, 100) for mid in self.motor_ids]
        self.send_packet_callback((AX_GOAL_SPEED, mcvs))    # set speeds of all motors to 200
        
        mcvs = zip(self.motor_ids, initial_positions)
        self.send_packet_callback((AX_GOAL_POSITION, mcvs)) # set initial pose
         
    def start(self):
        self.running = True
        self.joint_state_pub = rospy.Publisher('arm_controller/state', JointStateList)
        self.motor_states_sub = rospy.Subscriber('motor_states', MotorStateList, self.process_motor_states)
        self.joint_move_sub = rospy.Subscriber('arm_controller/joint_move', ArmJointMove, self.do_joint_move)
        
    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.motor_states_sub.unregister()
        self.joint_move_sub.unregister()
        
    def process_motor_states(self, state_list):
        if self.running:
            states = filter(lambda state: state.id in self.motor_ids, state_list.motor_states)
            if states:
                joints = []
                for state in states:
                    if state.id in self.ids_to_joints:
                        joint = self.ids_to_joints[state.id]
                        joints.append(JointState(joint.name,
                                                 joint.raw_position_to_angle(state.position),
                                                 (state.speed / AX_TICKS) * AX_MAX_SPEED_DEG,
                                                 filter(None, [joint.master_id, joint.inverse_id]),
                                                 state.moving))
                self.joint_state_pub.publish(joints)
                
    def do_joint_move(self, move):
        try:
            joint = self.name_to_joint[move.joint_name]
        except KeyError, ke:
            rospy.logwarn('%s is not a valid joint name' % move.joint_name)
        else:
            self.send_packet_callback((AX_GOAL_POSITION, joint.angle_to_raw_positions(move.angle)))

