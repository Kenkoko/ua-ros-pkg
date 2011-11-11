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

from w2_object_manipulation_launch.actions_common import ARM_JOINTS
from w2_object_manipulation_launch.actions_common import GRIPPER_GROUP_NAME
from w2_object_manipulation_launch.actions_common import GRIPPER_LINK_FRAME
from w2_object_manipulation_launch.actions_common import GRIPPER_LINKS
from w2_object_manipulation_launch.actions_common import LIFT_POSITION
from w2_object_manipulation_launch.actions_common import move_arm_joint_goal
from w2_object_manipulation_launch.msg import LiftObjectAction
from w2_object_manipulation_launch.msg import LiftObjectResult

from arm_navigation_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionObjectOperation
from arm_navigation_msgs.msg import CollisionOperation
from arm_navigation_msgs.msg import LinkPadding
from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import OrderedCollisionOperations

from control_msgs.msg import FollowJointTrajectoryFeedback

from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.msg import GraspHandPostureExecutionGoal
from object_manipulation_msgs.srv import GraspStatus

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_capture.srv import StartAudioRecording
from ua_audio_capture.srv import StopAudioRecording


ACTION_NAME = 'lift_object'


class LiftObjectActionServer:
    def __init__(self):
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        self.start_audio_recording_srv = rospy.ServiceProxy('/audio_dump/start_audio_recording', StartAudioRecording)
        self.stop_audio_recording_srv = rospy.ServiceProxy('/audio_dump/stop_audio_recording', StopAudioRecording)
        
        self.posture_controller = SimpleActionClient('/wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
        self.move_arm_client = SimpleActionClient('/move_left_arm', MoveArmAction)
        
        self.attached_object_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
        self.action_server = SimpleActionServer(ACTION_NAME,
                                                LiftObjectAction,
                                                execute_cb=self.lift_object,
                                                auto_start=False)

    def initialize(self):
        rospy.loginfo('%s: waiting for wubble_grasp_status service' % ACTION_NAME)
        rospy.wait_for_service('/wubble_grasp_status')
        rospy.loginfo('%s: connected to wubble_grasp_status service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for wubble_gripper_grasp_action' % ACTION_NAME)
        self.posture_controller.wait_for_server()
        rospy.loginfo('%s: connected to wubble_gripper_grasp_action' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for audio_dump/start_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/start_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/start_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for audio_dump/stop_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/stop_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/stop_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for move_left_arm action server' % ACTION_NAME)
        self.move_arm_client.wait_for_server()
        rospy.loginfo('%s: connected to move_left_arm action server' % ACTION_NAME)
        
        self.action_server.start()

    def open_gripper(self):
        pg = GraspHandPostureExecutionGoal()
        pg.goal = GraspHandPostureExecutionGoal.RELEASE
        
        self.posture_controller.send_goal(pg)
        self.posture_controller.wait_for_result()

    def lift_object(self, goal):
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: preempted' % ACTION_NAME)
            self.action_server.set_preempted()
            
        collision_support_surface_name = goal.collision_support_surface_name
        
        # disable collisions between grasped object and table
        collision_operation1 = CollisionOperation()
        collision_operation1.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        collision_operation1.object2 = collision_support_surface_name
        collision_operation1.operation = CollisionOperation.DISABLE
        
        # disable collisions between gripper and table
        collision_operation2 = CollisionOperation()
        collision_operation2.object1 = GRIPPER_GROUP_NAME
        collision_operation2.object2 = collision_support_surface_name
        collision_operation2.operation = CollisionOperation.DISABLE
        
        ordered_collision_operations = OrderedCollisionOperations()
        ordered_collision_operations.collision_operations = [collision_operation1, collision_operation2]
        
        gripper_paddings = [LinkPadding(l,0.0) for l in GRIPPER_LINKS]
        
        # this is a hack to make arm lift an object faster
        obj = AttachedCollisionObject()
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = GRIPPER_LINK_FRAME
        obj.object.operation.operation = CollisionObjectOperation.REMOVE
        obj.object.id = collision_support_surface_name
        obj.link_name = GRIPPER_LINK_FRAME
        obj.touch_links = GRIPPER_LINKS
        self.attached_object_pub.publish(obj)
        
        current_state = rospy.wait_for_message('l_arm_controller/state', FollowJointTrajectoryFeedback)
        joint_names = current_state.joint_names
        joint_positions = current_state.actual.positions
        start_angles = [joint_positions[joint_names.index(jn)] for jn in ARM_JOINTS]
        start_angles[0] = start_angles[0] - 0.3  # move shoulder up a bit
        
        if not move_arm_joint_goal(self.move_arm_client,
                                   ARM_JOINTS,
                                   start_angles,
                                   link_padding=gripper_paddings,
                                   collision_operations=ordered_collision_operations):
            self.action_server.set_aborted()
            return
            
        self.start_audio_recording_srv(InfomaxAction.LIFT, goal.category_id)
        
        if move_arm_joint_goal(self.move_arm_client,
                               ARM_JOINTS,
                               LIFT_POSITION,
                               link_padding=gripper_paddings,
                               collision_operations=ordered_collision_operations):
            # check if are still holding an object after lift is done
            grasp_status = self.get_grasp_status_srv()
            
            # if the object is still in hand after lift is done we are good
            if grasp_status.is_hand_occupied:
                sound_msg = self.stop_audio_recording_srv(True)
                self.action_server.set_succeeded(LiftObjectResult(sound_msg.recorded_sound))
                return
                
        self.stop_audio_recording_srv(False)
        rospy.logerr('%s: attempted lift failed' % ACTION_NAME)
        self.action_server.set_aborted()
        return


if __name__ == '__main__':
    rospy.init_node('lift_object_action_server', anonymous=True)
    loas = LiftObjectActionServer()
    loas.initialize()
    rospy.spin()

