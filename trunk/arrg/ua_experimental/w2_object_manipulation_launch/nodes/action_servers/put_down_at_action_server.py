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


import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from actionlib import SimpleActionClient
from actionlib import SimpleActionServer

from w2_object_manipulation_launch.actions_common import ARM_GROUP_NAME
from w2_object_manipulation_launch.actions_common import ARM_LINKS
from w2_object_manipulation_launch.actions_common import GRIPPER_GROUP_NAME
from w2_object_manipulation_launch.actions_common import GRIPPER_LINK_FRAME
from w2_object_manipulation_launch.actions_common import GRIPPER_LINKS
from w2_object_manipulation_launch.actions_common import move_arm_joint_goal
from w2_object_manipulation_launch.msg import PutDownAtAction
from w2_object_manipulation_launch.msg import PutDownAtResult

from arm_navigation_msgs.msg import ArmNavigationErrorCodes
from arm_navigation_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionObjectOperation
from arm_navigation_msgs.msg import CollisionOperation
from arm_navigation_msgs.msg import LinkPadding
from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import OrderedCollisionOperations
from arm_navigation_msgs.srv import SetPlanningSceneDiff

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32

from object_manipulation_msgs.msg import GraspPlanningErrorCode
from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.msg import GraspHandPostureExecutionGoal
from object_manipulation_msgs.srv import GraspPlanning
from object_manipulation_msgs.srv import GraspPlanningRequest
from object_manipulation_msgs.srv import GraspStatus
from object_manipulation_msgs.srv import FindClusterBoundingBox
from object_manipulation_msgs.srv import FindClusterBoundingBoxRequest
from object_manipulation_msgs.srv import FindClusterBoundingBoxResponse

from kinematics_msgs.srv import GetKinematicSolverInfo
from kinematics_msgs.srv import GetPositionIK
from kinematics_msgs.srv import GetPositionIKRequest

from sensor_msgs.msg import JointState

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_capture.srv import StartAudioRecording
from ua_audio_capture.srv import StopAudioRecording


ACTION_NAME = 'put_down_at'


class PutDownAtActionServer:
    def __init__(self):
        self.start_audio_recording_srv = rospy.ServiceProxy('/audio_dump/start_audio_recording', StartAudioRecording)
        self.stop_audio_recording_srv = rospy.ServiceProxy('/audio_dump/stop_audio_recording', StopAudioRecording)
        self.grasp_planning_srv = rospy.ServiceProxy('/GraspPlanning', GraspPlanning)
        self.get_solver_info_srv = rospy.ServiceProxy('/wubble2_left_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
        self.get_ik_srv = rospy.ServiceProxy('/wubble2_left_arm_kinematics/get_ik', GetPositionIK)
        self.set_planning_scene_diff_client = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        self.find_cluster_bbox = rospy.ServiceProxy('/find_cluster_bounding_box', FindClusterBoundingBox)
        
        self.posture_controller = SimpleActionClient('/wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
        self.move_arm_client = SimpleActionClient('/move_left_arm', MoveArmAction)
        
        self.attached_object_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
        self.action_server = SimpleActionServer(ACTION_NAME,
                                                PutDownAtAction,
                                                execute_cb=self.put_down_at,
                                                auto_start=False)

    def initialize(self):
        rospy.loginfo('%s: waiting for audio_dump/start_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/start_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/start_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for audio_dump/stop_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/stop_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/stop_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for GraspPlanning service' % ACTION_NAME)
        rospy.wait_for_service('/GraspPlanning')
        rospy.loginfo('%s: connected to GraspPlanning service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for wubble2_left_arm_kinematics/get_ik_solver_info service' % ACTION_NAME)
        rospy.wait_for_service('/wubble2_left_arm_kinematics/get_ik_solver_info')
        rospy.loginfo('%s: connected to wubble2_left_arm_kinematics/get_ik_solver_info service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for wubble2_left_arm_kinematics/get_ik service' % ACTION_NAME)
        rospy.wait_for_service('/wubble2_left_arm_kinematics/get_ik')
        rospy.loginfo('%s: connected to wubble2_left_arm_kinematics/get_ik service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for environment_server/set_planning_scene_diff service' % ACTION_NAME)
        rospy.wait_for_service('/environment_server/set_planning_scene_diff')
        rospy.loginfo('%s: connected to set_planning_scene_diff service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for wubble_gripper_grasp_action' % ACTION_NAME)
        self.posture_controller.wait_for_server()
        rospy.loginfo('%s: connected to wubble_gripper_grasp_action' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for move_left_arm action server' % ACTION_NAME)
        self.move_arm_client.wait_for_server()
        rospy.loginfo('%s: connected to move_left_arm action server' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for wubble_grasp_status service' % ACTION_NAME)
        rospy.wait_for_service('/wubble_grasp_status')
        rospy.loginfo('%s: connected to wubble_grasp_status service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for find_cluster_bounding_box service' % ACTION_NAME)
        rospy.wait_for_service('/find_cluster_bounding_box')
        rospy.loginfo('%s: connected to find_cluster_bounding_box service' % ACTION_NAME)
        
        self.action_server.start()

    def find_ik_for_grasping_pose(self, pose_stamped):
        solver_info = self.get_solver_info_srv()
        arm_joints = solver_info.kinematic_solver_info.joint_names
        
        req = GetPositionIKRequest()
        req.timeout = rospy.Duration(5.0)
        req.ik_request.ik_link_name = GRIPPER_LINK_FRAME
        req.ik_request.pose_stamped = pose_stamped
        
        try:
            current_state = rospy.wait_for_message('/joint_states', JointState, 2.0)
            
            req.ik_request.ik_seed_state.joint_state.name = arm_joints
            req.ik_request.ik_seed_state.joint_state.position = [current_state.position[current_state.name.index(j)] for j in arm_joints]
            
            if not self.set_planning_scene_diff_client():
                rospy.logerr('%s: Find IK for Grasp: failed to set planning scene diff' % ACTION_NAME)
                return None
                
            ik_result = self.get_ik_srv(req)
            
            if ik_result.error_code.val == ArmNavigationErrorCodes.SUCCESS:
                return ik_result.solution
            else:
                rospy.logerr('%s: failed to find an IK for requested grasping pose, aborting' % ACTION_NAME)
                return None
        except:
            rospy.logerr('%s: timed out waiting for joint state' % ACTION_NAME)
            return None

    def find_grasp_pose(self, target, collision_object_name='', collision_support_surface_name=''):
        """
        target = GraspableObject
        collision_object_name = name of target in collision map
        collision_support_surface_name = name of surface target is touching
        """
        req = GraspPlanningRequest()
        req.arm_name = ARM_GROUP_NAME
        req.target = target
        req.collision_object_name = collision_object_name
        req.collision_support_surface_name = collision_support_surface_name
        
        rospy.loginfo('%s: trying to find a good grasp for graspable object %s' % (ACTION_NAME, collision_object_name))
        grasping_result = self.grasp_planning_srv(req)
        
        if grasping_result.error_code.value != GraspPlanningErrorCode.SUCCESS:
            rospy.logerr('%s: unable to find a feasable grasp, aborting' % ACTION_NAME)
            return None
            
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = grasping_result.grasps[0].grasp_posture.header.frame_id
        pose_stamped.pose = grasping_result.grasps[0].grasp_pose
        
        rospy.loginfo('%s: found good grasp, looking for corresponding IK' % ACTION_NAME)
        
        return self.find_ik_for_grasping_pose(pose_stamped)


    def open_gripper(self):
        pg = GraspHandPostureExecutionGoal()
        pg.goal = GraspHandPostureExecutionGoal.RELEASE
        
        self.posture_controller.send_goal(pg)
        self.posture_controller.wait_for_result()


    def close_gripper(self):
        pg = GraspHandPostureExecutionGoal()
        pg.goal = GraspHandPostureExecutionGoal.GRASP
        
        self.posture_controller.send_goal(pg)
        self.posture_controller.wait_for_result()
        
        rospy.sleep(1)
        grasp_status = self.get_grasp_status_srv()
        return grasp_status.is_hand_occupied


    def find_cluster_bounding_box_center(self, cluster):
        fcbb = FindClusterBoundingBoxRequest()
        fcbb.cluster = cluster
        
        res = self.find_cluster_bbox(fcbb)
        
        if res.error_code == FindClusterBoundingBoxResponse.TF_ERROR:
            rospy.logerr('Unable to find bounding box for requested point cluster')
            return None
            
        return res.pose.pose.position


    def put_down_at(self, goal):
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: preempted' % ACTION_NAME)
            self.action_server.set_preempted()
            
        bbox_center = self.find_cluster_bounding_box_center(goal.graspable_object.cluster)
        if not bbox_center: self.action_server.set_aborted()
        
        goal.target_location.z = bbox_center.z
        x_dist = goal.target_location.x - bbox_center.x
        y_dist = goal.target_location.y - bbox_center.y
        
        target = goal.graspable_object
        target.cluster.points = [Point32(p.x+x_dist, p.y+y_dist, p.z) for p in target.cluster.points]
        
        collision_object_name = goal.collision_object_name
        collision_support_surface_name = goal.collision_support_surface_name
        
        ik_solution = self.find_grasp_pose(target, collision_object_name, collision_support_surface_name)
        
        if ik_solution:
            # disable collisions between gripper and target object
            ordered_collision_operations = OrderedCollisionOperations()
            collision_operation = CollisionOperation()
            collision_operation.object1 = collision_object_name
            collision_operation.object2 = GRIPPER_GROUP_NAME
            collision_operation.operation = CollisionOperation.DISABLE
            
            # disable collisions between gripper and table
            collision_operation2 = CollisionOperation()
            collision_operation2.object1 = collision_support_surface_name
            collision_operation2.object2 = GRIPPER_GROUP_NAME
            collision_operation2.operation = CollisionOperation.DISABLE
            collision_operation2.penetration_distance = 0.01
            
            # disable collisions between arm and table
            collision_operation3 = CollisionOperation()
            collision_operation3.object1 = collision_support_surface_name
            collision_operation3.object2 = ARM_GROUP_NAME
            collision_operation3.operation = CollisionOperation.DISABLE
            collision_operation3.penetration_distance = 0.01
            
            ordered_collision_operations.collision_operations = [collision_operation, collision_operation2, collision_operation3]
            
            # set gripper padding to 0
            gripper_paddings = [LinkPadding(l,0.0) for l in GRIPPER_LINKS]
            gripper_paddings.extend([LinkPadding(l,0.0) for l in ARM_LINKS])
            
            # move into pre-grasp pose
            if not move_arm_joint_goal(self.move_arm_client,
                                       ik_solution.joint_state.name,
                                       ik_solution.joint_state.position,
                                       link_padding=gripper_paddings,
                                       collision_operations=ordered_collision_operations):
                rospy.logerr('%s: attempted grasp failed' % ACTION_NAME)
                self.action_server.set_aborted()
                return
                
            # record put down sound with 0.5 second padding before and after
            self.start_audio_recording_srv(InfomaxAction.GRASP, goal.category_id)
            rospy.sleep(0.5)
            self.open_gripper()
            rospy.sleep(0.5)
            sound_msg = self.stop_audio_recording_srv(True)
            
            obj = AttachedCollisionObject()
            obj.object.header.stamp = rospy.Time.now()
            obj.object.header.frame_id = GRIPPER_LINK_FRAME
            obj.object.operation.operation = CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
            obj.object.id = collision_object_name
            obj.link_name = GRIPPER_LINK_FRAME
            obj.touch_links = GRIPPER_LINKS
            
            self.attached_object_pub.publish(obj)
            self.action_server.set_succeeded(PutDownAtResult(sound_msg.recorded_sound))
            return
            
        self.stop_audio_recording_srv(False)
        rospy.logerr('%s: attempted put down failed' % ACTION_NAME)
        self.action_server.set_aborted()


if __name__ == '__main__':
    rospy.init_node('put_down_at_action_server', anonymous=True)
    pdaas = PutDownAtActionServer()
    pdaas.initialize()
    rospy.spin()

