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
from w2_object_manipulation_launch.actions_common import ARM_GROUP_NAME
from w2_object_manipulation_launch.actions_common import GRIPPER_GROUP_NAME
from w2_object_manipulation_launch.actions_common import GRIPPER_LINK_FRAME
from w2_object_manipulation_launch.actions_common import GRIPPER_LINKS
from w2_object_manipulation_launch.actions_common import PLACE_POSITION
from w2_object_manipulation_launch.actions_common import move_arm_joint_goal
from w2_object_manipulation_launch.msg import PlaceObjectAction
from w2_object_manipulation_launch.msg import PlaceObjectResult

from arm_navigation_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionObjectOperation
from arm_navigation_msgs.msg import CollisionOperation
from arm_navigation_msgs.msg import LinkPadding
from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import OrderedCollisionOperations

from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.msg import GraspHandPostureExecutionGoal
from object_manipulation_msgs.srv import GraspStatus

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_capture.srv import StartAudioRecording
from ua_audio_capture.srv import StopAudioRecording


ACTION_NAME = 'place_object'


class PlaceObjectActionServer:
    def __init__(self):
        self.start_audio_recording_srv = rospy.ServiceProxy('/audio_dump/start_audio_recording', StartAudioRecording)
        self.stop_audio_recording_srv = rospy.ServiceProxy('/audio_dump/stop_audio_recording', StopAudioRecording)
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        
        self.posture_controller = SimpleActionClient('/wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
        self.move_arm_client = SimpleActionClient('/move_left_arm', MoveArmAction)
        
        self.attached_object_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
        self.action_server = SimpleActionServer(ACTION_NAME,
                                                PlaceObjectAction,
                                                execute_cb=self.place_object,
                                                auto_start=False)

    def initialize(self):
        rospy.loginfo('%s: waiting for audio_dump/start_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/start_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/start_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for audio_dump/stop_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/stop_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/stop_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for wubble_gripper_grasp_action' % ACTION_NAME)
        self.posture_controller.wait_for_server()
        rospy.loginfo('%s: connected to wubble_gripper_grasp_action' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for move_left_arm action server' % ACTION_NAME)
        self.move_arm_client.wait_for_server()
        rospy.loginfo('%s: connected to move_left_arm action server' % ACTION_NAME)
        
        self.action_server.start()

    def open_gripper(self):
        pg = GraspHandPostureExecutionGoal()
        pg.goal = GraspHandPostureExecutionGoal.RELEASE
        
        self.posture_controller.send_goal(pg)
        self.posture_controller.wait_for_result()

    def place_object(self, goal):
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: preempted' % ACTION_NAME)
            self.action_server.set_preempted()
            
        collision_object_name = goal.collision_object_name
        collision_support_surface_name = goal.collision_support_surface_name
        
        # check that we have something in hand before placing it
        grasp_status = self.get_grasp_status_srv()
        
        # if the object is still in hand after lift is done we are good
        if not grasp_status.is_hand_occupied:
            rospy.logerr('%s: gripper empty, nothing to place' % ACTION_NAME)
            self.action_server.set_aborted()
            return
            
        gripper_paddings = [LinkPadding(l,0.0) for l in GRIPPER_LINKS]
        
        # disable collisions between attached object and table
        collision_operation1 = CollisionOperation()
        collision_operation1.object1 = CollisionOperation.COLLISION_SET_ATTACHED_OBJECTS
        collision_operation1.object2 = collision_support_surface_name
        collision_operation1.operation = CollisionOperation.DISABLE
        
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
        
        ordered_collision_operations = OrderedCollisionOperations()
        ordered_collision_operations.collision_operations = [collision_operation1, collision_operation2, collision_operation3]
        
        self.start_audio_recording_srv(InfomaxAction.PLACE, goal.category_id)
        
        if move_arm_joint_goal(self.move_arm_client,
                               ARM_JOINTS,
                               PLACE_POSITION,
                               link_padding=gripper_paddings,
                               collision_operations=ordered_collision_operations):
            sound_msg = None
            grasp_status = self.get_grasp_status_srv()
            
            self.open_gripper()
            rospy.sleep(0.5)
            
            # if after lowering arm gripper is still holding an object life's good
            if grasp_status.is_hand_occupied:
                sound_msg = self.stop_audio_recording_srv(True)
            else:
                self.stop_audio_recording_srv(False)
                
            obj = AttachedCollisionObject()
            obj.object.header.stamp = rospy.Time.now()
            obj.object.header.frame_id = GRIPPER_LINK_FRAME
            obj.object.operation.operation = CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
            obj.object.id = collision_object_name
            obj.link_name = GRIPPER_LINK_FRAME
            obj.touch_links = GRIPPER_LINKS
            self.attached_object_pub.publish(obj)
            
            if sound_msg:
                self.action_server.set_succeeded(PlaceObjectResult(sound_msg.recorded_sound))
                return
            else:
                self.action_server.set_aborted()
                return
                
        self.stop_audio_recording_srv(False)
        rospy.logerr('%s: attempted place failed' % ACTION_NAME)
        self.action_server.set_aborted()


if __name__ == '__main__':
    rospy.init_node('place_object_action_server', anonymous=True)
    poas = PlaceObjectActionServer()
    poas.initialize()
    rospy.spin()

