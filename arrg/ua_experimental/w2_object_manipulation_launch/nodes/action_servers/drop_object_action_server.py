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

from w2_object_manipulation_launch.actions_common import GRIPPER_LINK_FRAME
from w2_object_manipulation_launch.actions_common import GRIPPER_LINKS
from w2_object_manipulation_launch.msg import DropObjectAction
from w2_object_manipulation_launch.msg import DropObjectResult

from arm_navigation_msgs.msg import AttachedCollisionObject
from arm_navigation_msgs.msg import CollisionObjectOperation

from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.msg import GraspHandPostureExecutionGoal
from object_manipulation_msgs.srv import GraspStatus

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_capture.srv import StartAudioRecording
from ua_audio_capture.srv import StopAudioRecording


ACTION_NAME = 'drop_object'


class DropObjectActionServer:
    def __init__(self):
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        self.start_audio_recording_srv = rospy.ServiceProxy('/audio_dump/start_audio_recording', StartAudioRecording)
        self.stop_audio_recording_srv = rospy.ServiceProxy('/audio_dump/stop_audio_recording', StopAudioRecording)
        
        self.posture_controller = SimpleActionClient('/wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
        self.attached_object_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject)
        self.action_server = SimpleActionServer(ACTION_NAME,
                                                DropObjectAction,
                                                execute_cb=self.drop_object,
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
        
        self.action_server.start()

    def open_gripper(self):
        pg = GraspHandPostureExecutionGoal()
        pg.goal = GraspHandPostureExecutionGoal.RELEASE
        
        self.posture_controller.send_goal(pg)
        self.posture_controller.wait_for_result()

    def drop_object(self, goal):
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: preempted' % ACTION_NAME)
            self.action_server.set_preempted()
            
        # check that we have something in hand before dropping it
        grasp_status = self.get_grasp_status_srv()
        
        # if the object is still in hand after lift is done we are good
        if not grasp_status.is_hand_occupied:
            rospy.logerr('%s: gripper empty, nothing to drop' % ACTION_NAME)
            self.action_server.set_aborted()
            return
            
        # record sound padded by 0.5 seconds from start and 1.5 seconds from the end
        self.start_audio_recording_srv(InfomaxAction.DROP, goal.category_id)
        rospy.sleep(0.5)
        self.open_gripper()
        rospy.sleep(1.5)
        
        # check if gripper actually opened first
        sound_msg = None
        grasp_status = self.get_grasp_status_srv()
        
        # if there something in the gripper - drop failed
        if grasp_status.is_hand_occupied:
            self.stop_audio_recording_srv(False)
        else:
            sound_msg = self.stop_audio_recording_srv(True)
            
        # delete the object that we just dropped, we don't really know where it will land
        obj = AttachedCollisionObject()
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = GRIPPER_LINK_FRAME
        obj.object.operation.operation = CollisionObjectOperation.REMOVE
        obj.object.id = goal.collision_object_name
        obj.link_name = GRIPPER_LINK_FRAME
        obj.touch_links = GRIPPER_LINKS
        
        self.attached_object_pub.publish(obj)
        
        if sound_msg:
            self.action_server.set_succeeded(DropObjectResult(sound_msg.recorded_sound))
        else:
            self.action_server.set_aborted()


if __name__ == '__main__':
    rospy.init_node('ready_arm_action_server', anonymous=True)
    raas = DropObjectActionServer()
    raas.initialize()
    rospy.spin()

