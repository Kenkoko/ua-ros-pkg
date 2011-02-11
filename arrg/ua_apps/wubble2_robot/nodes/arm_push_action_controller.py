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

import roslib; roslib.load_manifest('wubble2_robot')
import rospy

from move_arm_msgs.msg import MoveArmAction
from move_arm_msgs.msg import MoveArmGoal
from motion_planning_msgs.msg import JointConstraint

from ua_controller_msgs.msg import JointState

import actionlib
from actionlib_msgs.msg import GoalStatus

from wubble2_robot.msg import WubbleArmPushAction
from wubble2_robot.msg import WubbleArmPushGoal
from wubble2_robot.msg import WubbleGripperAction
from wubble2_robot.msg import WubbleGripperGoal


import math
import numpy
import tf

from motion_planning_msgs.srv import GetMotionPlanRequest
from motion_planning_msgs.srv import GetMotionPlanResponse
from motion_planning_msgs.srv import GetMotionPlan

from motion_planning_msgs.msg import PositionConstraint
from motion_planning_msgs.msg import OrientationConstraint
from motion_planning_msgs.msg import ArmNavigationErrorCodes
from motion_planning_msgs.msg import OrderedCollisionOperations
from motion_planning_msgs.msg import CollisionOperation

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from interpolated_ik_motion_planner.srv import SetInterpolatedIKMotionPlanParams

from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint



class GripperActionController():
    def __init__(self):
        rospy.init_node('arm_push_action_controller', anonymous=True)
        self.arm_joints = ( 'shoulder_pitch_joint',
                            'shoulder_yaw_joint',
                            'shoulder_roll_joint',
                            'elbow_pitch_joint',
                            'wrist_roll_joint',
                            'wrist_pitch_joint',
                            'wrist_yaw_joint' )
                            
        self.arm_ready_pos = (0.40, -0.98, -0.11, 0.61, 0.21, -0.47, 0.18)
        
        self.move_arm_client = actionlib.SimpleActionClient('move_left_arm', MoveArmAction)
        self.move_arm_client.wait_for_server()
        
        self.traj_client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self.traj_client.wait_for_server()
        
        self.gripper_client = actionlib.SimpleActionClient('wubble_gripper_action', WubbleGripperAction)
        self.gripper_client.wait_for_server()
        
        self.action_server = actionlib.SimpleActionServer('wubble_arm_push_action', WubbleArmPushAction, execute_cb=self.process_arm_push_action)
        rospy.loginfo('arm_push_action_controller: ready to accept goals')

    def process_arm_push_action(self, req):
        mag = MoveArmGoal()
        
        mag.motion_plan_request.group_name = 'left_arm'
        mag.motion_plan_request.num_planning_attempts = 1
        mag.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
        
        mag.motion_plan_request.planner_id= ''
        mag.planner_service_name = 'ompl_planning/plan_kinematic_path'
        
        for idx, joint_name in enumerate(self.arm_joints):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = self.arm_ready_pos[idx]
            jc.tolerance_below = 0.1
            jc.tolerance_above = 0.1
            mag.motion_plan_request.goal_constraints.joint_constraints.append(jc)
            
        self.move_arm_client.send_goal(mag)
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        
        if not finished_within_time:
            self.move_arm_client.cancel_goal()
            rospy.loginfo('Timed out trying to get to ready position')
            self.action_server.set_aborted()
            return
        else:
            state = self.move_arm_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo('Action finished: %s', str(state))
            else:
                rospy.logerr('Action failed: %s', str(state))
                self.action_server.set_aborted()
                return
                
        self.open_gripper()
        self.save_sound = True
        
        # side grasp
        msg = rospy.wait_for_message('l_arm_controller/state', JointTrajectoryControllerState)
        start_angles = msg.actual.positions
        approachpos = [0.0, 0.0, 0.0]
        approachquat = [0.00000, 0.00000, 0.70711, -0.70711]
        grasppos = [0.0, -0.5, 0.0]
        graspquat = approachquat[:]
        frame_id = 'L7_wrist_yaw_link'
        self.check_cartesian_path_lists(approachpos, approachquat, grasppos, graspquat, start_angles, frame=frame_id)
        
        #self.close_gripper()
        self.action_server.set_succeeded()

    def open_gripper(self):
        # Creates a goal to send to the action server.
        goal = WubbleGripperGoal()
        goal.command = WubbleGripperGoal.OPEN_GRIPPER
        goal.torque_limit = 1.0
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def close_gripper(self):
        # Creates a goal to send to the action server.
        goal = WubbleGripperGoal()
        goal.command = WubbleGripperGoal.CLOSE_GRIPPER
        goal.torque_limit = 0.2
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    #pretty-print list to string
    def pplist(self, list_to_print):
        return ' '.join(['%5.3f'%x for x in list_to_print])

    #call the service
    def call_get_interpolated_ik_motion_plan(self, start_pose, goal_pose, start_angles, joint_names, pos_spacing = 0.01, \
                                             rot_spacing = 0.1, consistent_angle = math.pi/9, collision_aware = 1, \
                                             collision_check_resolution = 1, steps_before_abort = -1, num_steps = 0, \
                                             ordered_collision_operations = None, frame = 'base_footprint', start_from_end = 0, \
                                             max_joint_vels = [0.75]*7, max_joint_accs = [8.0]*7):
        print "waiting for l_interpolated_ik_motion_plan service"
        rospy.wait_for_service("l_interpolated_ik_motion_plan")
        print "l_interpolated_ik_motion_plan service found"
        
        try:
            serv = rospy.ServiceProxy("l_interpolated_ik_motion_plan_set_params", SetInterpolatedIKMotionPlanParams)
            res = serv(num_steps, consistent_angle, collision_check_resolution, steps_before_abort, pos_spacing, rot_spacing, collision_aware, start_from_end, max_joint_vels, max_joint_accs)
        except rospy.ServiceException, e:
            print "error when calling l_interpolated_ik_motion_plan_set_params: %s"%e  
            return 0
            
        req = GetMotionPlanRequest()
        req.motion_plan_request.start_state.joint_state.name = joint_names
        req.motion_plan_request.start_state.joint_state.position = start_angles
        req.motion_plan_request.start_state.multi_dof_joint_state.pose = start_pose.pose
        req.motion_plan_request.start_state.multi_dof_joint_state.child_frame_id = 'L7_wrist_yaw_link'
        req.motion_plan_request.start_state.multi_dof_joint_state.frame_id = start_pose.header.frame_id
        
        pos_constraint = PositionConstraint()
        pos_constraint.position = goal_pose.pose.position
        pos_constraint.header.frame_id = goal_pose.header.frame_id
        req.motion_plan_request.goal_constraints.position_constraints = [pos_constraint,]
        
        orient_constraint = OrientationConstraint()
        orient_constraint.orientation = goal_pose.pose.orientation
        orient_constraint.header.frame_id = goal_pose.header.frame_id
        req.motion_plan_request.goal_constraints.orientation_constraints = [orient_constraint,]
        
        if ordered_collision_operations != None:
            req.motion_plan_request.ordered_collision_operations = ordered_collision_operations
            
        try:
            serv = rospy.ServiceProxy("l_interpolated_ik_motion_plan", GetMotionPlan)
            res = serv(req)
        except rospy.ServiceException as e:
            print "error when calling l_interpolated_ik_motion_plan: %s"%e  
            return 0
            
        error_code_dict = { ArmNavigationErrorCodes.SUCCESS: 0,
                            ArmNavigationErrorCodes.COLLISION_CONSTRAINTS_VIOLATED: 1,
                            ArmNavigationErrorCodes.PATH_CONSTRAINTS_VIOLATED: 2,
                            ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED: 3,
                            ArmNavigationErrorCodes.PLANNING_FAILED: 4 }
        
        trajectory_len = len(res.trajectory.joint_trajectory.points)
        trajectory = [res.trajectory.joint_trajectory.points[i].positions for i in range(trajectory_len)]
        vels = [res.trajectory.joint_trajectory.points[i].velocities for i in range(trajectory_len)]
        times = [res.trajectory.joint_trajectory.points[i].time_from_start for i in range(trajectory_len)]
        error_codes = [error_code_dict[error_code.val] for error_code in res.trajectory_error_codes]
        
        rospy.loginfo("trajectory:")
        
        for ind in range(len(trajectory)):
            rospy.loginfo("error code "+ str(error_codes[ind]) + " pos : " + self.pplist(trajectory[ind]))
        
        rospy.loginfo("")
        
        for ind in range(len(trajectory)):
            rospy.loginfo("time: " + "%5.3f  "%times[ind].to_sec() + "vels: " + self.pplist(vels[ind]))
            
        goal = JointTrajectoryGoal()
        for i, p in enumerate(res.trajectory.joint_trajectory.points):
            if res.trajectory_error_codes[i].val == ArmNavigationErrorCodes.SUCCESS:
                goal.trajectory.points.append(p)
        goal.trajectory.joint_names = res.trajectory.joint_trajectory.joint_names
        goal.trajectory.points = goal.trajectory.points[1:] # skip the 0 velocity point
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        self.traj_client.send_goal(goal)
        self.traj_client.wait_for_result()

    #create a PoseStamped message
    def create_pose_stamped(self, pose, frame_id = 'base_footprint'):
        m = PoseStamped()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time()
        m.pose = Pose(Point(*pose[0:3]), Quaternion(*pose[3:7]))
        return m

    #convert to PoseStamped then call the service
    def check_cartesian_path_lists(self, approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing = 0.01, \
                                   rot_spacing = 0.1, consistent_angle = math.pi/9., collision_aware = 1, \
                                   collision_check_resolution = 1, steps_before_abort = -1, num_steps = 0, \
                                   ordered_collision_operations = None, frame = 'base_footprint'):
        
        start_pose = self.create_pose_stamped(approachpos+approachquat, frame)
        goal_pose = self.create_pose_stamped(grasppos+graspquat, frame)
        
        self.call_get_interpolated_ik_motion_plan(start_pose, goal_pose, start_angles, self.arm_joints, pos_spacing, rot_spacing, \
                                                  consistent_angle, collision_aware, collision_check_resolution, \
                                                  steps_before_abort, num_steps, ordered_collision_operations, frame)


if __name__ == '__main__':
    gac = GripperActionController()
    rospy.spin()

