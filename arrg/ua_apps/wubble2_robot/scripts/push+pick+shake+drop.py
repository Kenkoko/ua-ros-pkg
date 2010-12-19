#! /usr/bin/env python

# Copyright (c) 2010, Antons Rebguns
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
#
# Author: Antons Rebguns
#

PKG = 'wubble2_robot'
NAME = 'all_actions'

import roslib; roslib.load_manifest(PKG)
import rospy

from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from ua_audio_msgs.msg import TransformedStream

from actionlib import SimpleActionClient
from wubble2_robot.msg import *

class ObjectSoundCollector:
    def __init__(self):
        self.sound = []
        self.save_sound_flag = False
        self.sound_sub = rospy.Subscriber('transform_audio', TransformedStream, self.process_audio)
        
        self.push_client = SimpleActionClient('wubble_arm_push_action', WubbleArmPushAction)
        self.push_client.wait_for_server()
        
        self.gripper_client = SimpleActionClient('wubble_gripper_action', WubbleGripperAction)
        self.gripper_client.wait_for_server()
        
        self.traj_client = SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self.traj_client.wait_for_server()
        
        self.shake_client = SimpleActionClient('wubble_gripper_shake_action', WubbleGripperShakeAction)
        self.shake_client.wait_for_server()


    def process_audio(self, msg):
        if self.save_sound: self.sound.extend(msg.stream)

    def save_sound(self, action):
        # save the sound to file and ditch it from memory
        f = open('/tmp/%s_action_%f' % (action, rospy.Time.now().to_sec()), 'w')
        f.write(str(self.sound))
        f.close()

    def close_gripper(self):
        goal = WubbleGripperGoal()
        goal.command = WubbleGripperGoal.CLOSE_GRIPPER
        goal.torque_limit = 0.5
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def open_gripper(self):
        goal = WubbleGripperGoal()
        goal.command = WubbleGripperGoal.OPEN_GRIPPER
        goal.torque_limit = 1.0
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def lift(self):
        goal = JointTrajectoryGoal()

        # First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.append('shoulder_pitch_joint')
        goal.trajectory.joint_names.append('shoulder_yaw_joint')
        goal.trajectory.joint_names.append('shoulder_roll_joint')
        goal.trajectory.joint_names.append('elbow_pitch_joint')
        goal.trajectory.joint_names.append('wrist_roll_joint')
        goal.trajectory.joint_names.append('wrist_pitch_joint')
        goal.trajectory.joint_names.append('wrist_yaw_joint')

        point = JointTrajectoryPoint()
        point.positions  = [-1.10, -1.58, 0.53, -0.76, -1.13, -0.31, -0.18]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = rospy.Duration(3.0)
        goal.trajectory.points.append(point)

        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        self.traj_client.send_goal(goal)
        self.traj_client.wait_for_result()

    def push(self):
        goal = WubbleArmPushGoal()
        goal.side = WubbleArmPushGoal.RIGHT_PUSH
        
        self.save_sound_flag = True
        
        self.push_client.send_goal(goal)
        self.push_client.wait_for_result()
        
        self.save_sound_flag = False
        self.save_sound('push')
        self.sound = []

    def pick(self):
        self.save_sound_flag = True
        
        self.close_gripper()
        self.lift()
        
        self.save_sound_flag = False
        self.save_sound('pick')
        self.sound = []

    def shake(self):
        goal = WubbleGripperShakeGoal()
        goal.shake_number = 5
        
        self.save_sound_flag = True
        
        self.shake_client.send_goal(goal)
        self.shake_client.wait_for_result()
        
        self.save_sound_flag = False
        self.save_sound('shake')
        self.sound = []

    def drop(self):
        self.save_sound_flag = True
        
        self.open_gripper()
        
        rospy.sleep(0.5)
        self.save_sound_flag = False
        self.save_sound('drop')
        self.sound = []

if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        osc = ObjectSoundCollector()
        rospy.loginfo('Doing a Push action')
        osc.push()
        rospy.loginfo('Doing a PickUp action')
        osc.pick()
        rospy.loginfo('Doing a Shake action')
        osc.shake()
        rospy.loginfo('Doing a Drop action')
        osc.drop()
        
        rospy.sleep(2)
        rospy.loginfo('Closing gripper')
        osc.close_gripper()
    except rospy.ROSInterruptException:
        pass

