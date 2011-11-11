#!/usr/bin/env python

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


import math

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from actionlib import SimpleActionClient
from actionlib import SimpleActionServer
from actionlib_msgs.msg import GoalStatus

from w2_object_manipulation_launch.actions_common import ARM_LINKS
from w2_object_manipulation_launch.actions_common import ARM_JOINTS
from w2_object_manipulation_launch.actions_common import GRIPPER_GROUP_NAME
from w2_object_manipulation_launch.actions_common import GRIPPER_LINK_FRAME
from w2_object_manipulation_launch.actions_common import GRIPPER_LINKS
from w2_object_manipulation_launch.msg import PushObjectAction
from w2_object_manipulation_launch.msg import PushObjectResult

from arm_navigation_msgs.msg import ArmNavigationErrorCodes
from arm_navigation_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionObjectOperation
from arm_navigation_msgs.msg import CollisionOperation
from arm_navigation_msgs.srv import FilterJointTrajectory
from arm_navigation_msgs.srv import FilterJointTrajectoryRequest
from arm_navigation_msgs.msg import LinkPadding
from arm_navigation_msgs.msg import OrderedCollisionOperations
from arm_navigation_msgs.msg import OrientationConstraint
from arm_navigation_msgs.msg import PositionConstraint
from arm_navigation_msgs.srv import GetMotionPlan
from arm_navigation_msgs.srv import GetMotionPlanRequest
from arm_navigation_msgs.srv import SetPlanningSceneDiff

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryFeedback

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

from kinematics_msgs.srv import GetPositionFK
from kinematics_msgs.srv import GetPositionFKRequest

from interpolated_ik_motion_planner.srv import SetInterpolatedIKMotionPlanParams

from object_manipulation_msgs.srv import GraspStatus

from sensor_msgs.msg import JointState

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_capture.srv import StartAudioRecording
from ua_audio_capture.srv import StopAudioRecording


ACTION_NAME = 'push_object'


class PushObjectActionServer:
    def __init__(self):
        self.start_audio_recording_srv = rospy.ServiceProxy('/audio_dump/start_audio_recording', StartAudioRecording)
        self.stop_audio_recording_srv = rospy.ServiceProxy('/audio_dump/stop_audio_recording', StopAudioRecording)
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        self.interpolated_ik_params_srv = rospy.ServiceProxy('/l_interpolated_ik_motion_plan_set_params', SetInterpolatedIKMotionPlanParams)
        self.interpolated_ik_srv = rospy.ServiceProxy('/l_interpolated_ik_motion_plan', GetMotionPlan)
        self.set_planning_scene_diff_client = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
        self.get_fk_srv = rospy.ServiceProxy('/wubble2_left_arm_kinematics/get_fk', GetPositionFK)
        self.trajectory_filter_srv = rospy.ServiceProxy('/trajectory_filter_unnormalizer/filter_trajectory', FilterJointTrajectory)
        
        self.trajectory_controller = SimpleActionClient('/l_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.attached_object_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
        self.action_server = SimpleActionServer(ACTION_NAME,
                                                PushObjectAction,
                                                execute_cb=self.push_object,
                                                auto_start=False)


    def initialize(self):
        rospy.loginfo('%s: waiting for audio_dump/start_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/start_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/start_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for audio_dump/stop_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/stop_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/stop_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for l_interpolated_ik_motion_plan_set_params service' % ACTION_NAME)
        rospy.wait_for_service('/l_interpolated_ik_motion_plan_set_params')
        rospy.loginfo('%s: connected to l_interpolated_ik_motion_plan_set_params service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for l_interpolated_ik_motion_plan service' % ACTION_NAME)
        rospy.wait_for_service('/l_interpolated_ik_motion_plan')
        rospy.loginfo('%s: connected to l_interpolated_ik_motion_plan service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for environment_server/set_planning_scene_diff service' % ACTION_NAME)
        rospy.wait_for_service('/environment_server/set_planning_scene_diff')
        rospy.loginfo('%s: connected to set_planning_scene_diff service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for wubble2_left_arm_kinematics/get_fk service' % ACTION_NAME)
        rospy.wait_for_service('/wubble2_left_arm_kinematics/get_fk')
        rospy.loginfo('%s: connected to wubble2_left_arm_kinematics/get_fk service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for trajectory_filter_unnormalizer/filter_trajectory service' % ACTION_NAME)
        rospy.wait_for_service('/trajectory_filter_unnormalizer/filter_trajectory')
        rospy.loginfo('%s: connected to trajectory_filter_unnormalizer/filter_trajectory service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for l_arm_controller/follow_joint_trajectory' % ACTION_NAME)
        self.trajectory_controller.wait_for_server()
        rospy.loginfo('%s: connected to l_arm_controller/follow_joint_trajectory' % ACTION_NAME)
        
        self.action_server.start()


    def create_pose_stamped(self, pose, frame_id='base_link'):
        """
        Creates a PoseStamped message from a list of 7 numbers (first three are
        position and next four are orientation:
        pose = [px,py,pz, ox,oy,oz,ow]
        """
        m = PoseStamped()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time()
        m.pose.position = Point(*pose[0:3])
        m.pose.orientation = Quaternion(*pose[3:7])
        return m


    #pretty-print list to string
    def pplist(self, list_to_print):
        return ' '.join(['%5.3f'%x for x in list_to_print])


    def get_interpolated_ik_motion_plan(self, start_pose, goal_pose, start_angles, joint_names, pos_spacing=0.01,
                                        rot_spacing=0.1, consistent_angle=math.pi/9, collision_aware=True,
                                        collision_check_resolution=1, steps_before_abort=-1, num_steps=0,
                                        ordered_collision_operations=None, frame='base_footprint', start_from_end=0,
                                        max_joint_vels=[1.5]*7, max_joint_accs=[8.0]*7):
                                        
        res = self.interpolated_ik_params_srv(num_steps,
                                              consistent_angle,
                                              collision_check_resolution,
                                              steps_before_abort,
                                              pos_spacing,
                                              rot_spacing,
                                              collision_aware,
                                              start_from_end,
                                              max_joint_vels,
                                              max_joint_accs)
                                              
        req = GetMotionPlanRequest()
        req.motion_plan_request.start_state.joint_state.name = joint_names
        req.motion_plan_request.start_state.joint_state.position = start_angles
        req.motion_plan_request.start_state.multi_dof_joint_state.poses = [start_pose.pose]
        req.motion_plan_request.start_state.multi_dof_joint_state.child_frame_ids = [GRIPPER_LINK_FRAME]
        req.motion_plan_request.start_state.multi_dof_joint_state.frame_ids = [start_pose.header.frame_id]
        
        pos_constraint = PositionConstraint()
        pos_constraint.position = goal_pose.pose.position
        pos_constraint.header.frame_id = goal_pose.header.frame_id
        req.motion_plan_request.goal_constraints.position_constraints = [pos_constraint,]
        
        orient_constraint = OrientationConstraint()
        orient_constraint.orientation = goal_pose.pose.orientation
        orient_constraint.header.frame_id = goal_pose.header.frame_id
        req.motion_plan_request.goal_constraints.orientation_constraints = [orient_constraint,]
        
        #req.motion_plan_request.link_padding = [LinkPadding(l,0.0) for l in GRIPPER_LINKS]
        #req.motion_plan_request.link_padding.extend([LinkPadding(l,0.0) for l in ARM_LINKS])
        
        #if ordered_collision_operations is not None:
        #    req.motion_plan_request.ordered_collision_operations = ordered_collision_operations
            
        res = self.interpolated_ik_srv(req)
        return res


    def check_cartesian_path_lists(self, approachpos, approachquat, grasppos, graspquat, start_angles, pos_spacing=0.03,
                                   rot_spacing=0.1, consistent_angle=math.pi/7.0, collision_aware=True,
                                   collision_check_resolution=1, steps_before_abort=1, num_steps=0,
                                   ordered_collision_operations=None, frame='base_link'):
                                   
        start_pose = self.create_pose_stamped(approachpos+approachquat, frame)
        goal_pose = self.create_pose_stamped(grasppos+graspquat, frame)
        
        return self.get_interpolated_ik_motion_plan(start_pose, goal_pose, start_angles, ARM_JOINTS, pos_spacing, rot_spacing,
                                                    consistent_angle, collision_aware, collision_check_resolution,
                                                    steps_before_abort, num_steps, ordered_collision_operations, frame)


    def push_object(self, goal):
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: preempted' % ACTION_NAME)
            self.action_server.set_preempted()
            
        collision_object_name = goal.collision_object_name
        collision_support_surface_name = goal.collision_support_surface_name
        
        current_state = rospy.wait_for_message('l_arm_controller/state', FollowJointTrajectoryFeedback)
        start_angles = current_state.actual.positions
        
        full_state = rospy.wait_for_message('/joint_states', JointState)
        
        req = GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [GRIPPER_LINK_FRAME]
        req.robot_state.joint_state = full_state
        
        if not self.set_planning_scene_diff_client():
            rospy.logerr('%s: failed to set planning scene diff' % ACTION_NAME)
            self.action_server.set_aborted()
            return
            
        pose = self.get_fk_srv(req).pose_stamped[0].pose
        
        frame_id = 'base_link'
        
        approachpos =  [pose.position.x, pose.position.y, pose.position.z]
        approachquat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        
        push_distance = 0.40
        grasppos =  [pose.position.x, pose.position.y-push_distance, pose.position.z]
        graspquat = [0.00000, 0.00000, 0.70711, -0.70711]
        
        # attach object to gripper, they will move together
        obj = AttachedCollisionObject()
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = GRIPPER_LINK_FRAME
        obj.object.operation.operation = CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
        obj.object.id = collision_object_name
        obj.link_name = GRIPPER_LINK_FRAME
        obj.touch_links = GRIPPER_LINKS
        
        self.attached_object_pub.publish(obj)
        
        # disable collisions between attached object and table
        collision_operation1 = CollisionOperation()
        collision_operation1.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        collision_operation1.object2 = collision_support_surface_name
        collision_operation1.operation = CollisionOperation.DISABLE
        
        collision_operation2 = CollisionOperation()
        collision_operation2.object1 = collision_support_surface_name
        collision_operation2.object2 = GRIPPER_GROUP_NAME
        collision_operation2.operation = CollisionOperation.DISABLE
        collision_operation2.penetration_distance = 0.02
        
        ordered_collision_operations = OrderedCollisionOperations()
        ordered_collision_operations.collision_operations = [collision_operation1, collision_operation2]
        
        res = self.check_cartesian_path_lists(approachpos,
                                              approachquat,
                                              grasppos,
                                              graspquat,
                                              start_angles,
                                              frame=frame_id,
                                              ordered_collision_operations=ordered_collision_operations)
                                              
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
        
        # if even one code is not 0, abort
        if sum(error_codes) != 0:
            for ind in range(len(trajectory)):
                rospy.loginfo("error code "+ str(error_codes[ind]) + " pos : " + self.pplist(trajectory[ind]))
                
            rospy.loginfo("")
            
            for ind in range(len(trajectory)):
                rospy.loginfo("time: " + "%5.3f  "%times[ind].to_sec() + "vels: " + self.pplist(vels[ind]))
                
            rospy.logerr('%s: attempted push failed' % ACTION_NAME)
            self.action_server.set_aborted()
            return
            
        req = FilterJointTrajectoryRequest()
        req.trajectory = res.trajectory.joint_trajectory
        req.trajectory.points = req.trajectory.points[1:] # skip zero velocity point
        req.allowed_time = rospy.Duration(2.0)
        
        filt_res = self.trajectory_filter_srv(req)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = filt_res.trajectory
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        
        self.start_audio_recording_srv(InfomaxAction.PUSH, goal.category_id)
        rospy.sleep(0.5)
        
        self.trajectory_controller.send_goal(goal)
        self.trajectory_controller.wait_for_result()
        
        state = self.trajectory_controller.get_state()
        
        if state == GoalStatus.SUCCEEDED:
            rospy.sleep(0.5)
            sound_msg = self.stop_audio_recording_srv(True)
            self.action_server.set_succeeded(PushObjectResult(sound_msg.recorded_sound))
            return
            
        rospy.logerr('%s: attempted push failed' % ACTION_NAME)
        self.stop_audio_recording_srv(False)
        self.action_server.set_aborted()


if __name__ == '__main__':
    rospy.init_node('place_object_action_server', anonymous=True)
    poas = PushObjectActionServer()
    poas.initialize()
    rospy.spin()

