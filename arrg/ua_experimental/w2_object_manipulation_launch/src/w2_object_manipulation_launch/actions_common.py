# -*- coding: utf-8 -*-
#
# Copyright (c) 2011, Antons Rebguns
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# Neither the name of the <ORGANIZATION> nor the names of its contributors may
# be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import numpy as np

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from actionlib_msgs.msg import GoalStatus

from arm_navigation_msgs.msg import JointConstraint
from arm_navigation_msgs.msg import MoveArmGoal
from arm_navigation_msgs.msg import OrderedCollisionOperations

from control_msgs.msg import FollowJointTrajectoryFeedback


ARM_JOINTS = (
    'shoulder_pitch_joint',
    'shoulder_pan_joint',
    'upperarm_roll_joint',
    'elbow_flex_joint',
    'forearm_roll_joint',
    'wrist_pitch_joint',
    'wrist_roll_joint',
)

ARM_LINKS = (
    'L6_wrist_pitch_link',
    'L5_forearm_roll_link',
    'L4_elbow_flex_link',
    'L3_upperarm_roll_link',
    'L2_shoulder_pan_link',
    'L1_shoulder_pitch_link',
)

GRIPPER_LINKS = (
    'L9_right_finger_link',
    'L8_left_finger_link',
    'L7_wrist_roll_link',
)

#READY_POSITION = (-1.650, -1.465,  3.430, -0.970, -1.427,  0.337,  0.046)
READY_POSITION = (-1.946, -2.041,  0.135,  1.036, -2.045,  2.004, -0.158)
#LIFT_POSITION  = (-1.049, -1.241,  0.669, -0.960, -0.409, -0.072, -0.143)
LIFT_POSITION = (-1.946, -2.041,  0.135,  1.036, -2.045,  2.004, -0.158)
PLACE_POSITION = ( 0.261, -0.704,  1.470,  0.337,  0.910, -1.667, -0.026)
TUCK_POSITION  = (-1.751, -1.820, -0.084, -0.214, -1.815,  1.089,  0.031)

ARM_STATES = {
    'READY': READY_POSITION,
    'LIFT':  LIFT_POSITION,
    'PLACE': PLACE_POSITION,
    'TUCK':  TUCK_POSITION,
    'OTHER': None,
}

GRIPPER_LINK_FRAME = 'L7_wrist_roll_link'
GRIPPER_GROUP_NAME = 'l_end_effector'
ARM_GROUP_NAME = 'left_arm'


def lists_within_tolerance(list1, list2, tolerances):
    if list1 is None or list2 is None: return False
    l1 = np.asarray(list1)
    l2 = np.asarray(list2)
    
    # are all absolute differences smaller than provided tolerances?
    return (np.abs(l1 - l2) < tolerances).all()


def find_current_arm_state(tolerances=[0.04]*7):
    arm_state = rospy.wait_for_message('l_arm_controller/state', FollowJointTrajectoryFeedback, 2.0)
    joint_names = arm_state.joint_names
    joint_positions = arm_state.actual.positions
    
    # make sure we put the state in expected order of joints
    current_joint_positions = [joint_positions[joint_names.index(jn)] for jn in ARM_JOINTS]
    
    for state_name in ARM_STATES:
        desired_joint_positions = ARM_STATES[state_name]
        if lists_within_tolerance(desired_joint_positions, current_joint_positions, tolerances): return state_name


def is_arm_in_state(state_name, tolerances=[0.04]*7):
    current_joint_positions = rospy.wait_for_message('l_arm_controller/state', FollowJointTrajectoryFeedback, 2.0).actual.positions
    desired_joint_positions = ARM_STATES[state_name]
    return lists_within_tolerance(desired_joint_positions, current_joint_positions, tolerances)


def move_arm_joint_goal(move_arm_client,
                        joint_names,
                        joint_positions,
                        allowed_contacts=[],
                        link_padding=[],
                        collision_operations=OrderedCollisionOperations()):
    goal = MoveArmGoal()
    goal.planner_service_name = 'ompl_planning/plan_kinematic_path'
    goal.motion_plan_request.planner_id = ''
    goal.motion_plan_request.group_name = ARM_GROUP_NAME
    goal.motion_plan_request.num_planning_attempts = 3
    goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
    goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint(j, p, 0.1, 0.1, 0.0) for (j,p) in zip(joint_names,joint_positions)]
    
    goal.planning_scene_diff.allowed_contacts = allowed_contacts
    goal.planning_scene_diff.link_padding = link_padding
    goal.operations = collision_operations
    
    move_arm_client.send_goal(goal)
    finished_within_time = move_arm_client.wait_for_result(rospy.Duration(200.0))
    
    if not finished_within_time:
        move_arm_client.cancel_goal()
        rospy.logerr('timed out trying to achieve joint goal')
        return False
    else:
        state = move_arm_client.get_state()
        
        if state == GoalStatus.SUCCEEDED:
            return True
        else:
            rospy.logerr('failed to achieve joint goal (returned status code %s)' % str(state))
            return False

