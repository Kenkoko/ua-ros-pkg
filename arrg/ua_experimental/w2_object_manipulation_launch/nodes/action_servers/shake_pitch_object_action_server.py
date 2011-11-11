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

from actionlib import SimpleActionServer

from w2_object_manipulation_launch.msg import ShakePitchObjectAction
from w2_object_manipulation_launch.msg import ShakePitchObjectResult

from object_manipulation_msgs.srv import GraspStatus

from dynamixel_hardware_interface.msg import JointState as DynamixelJointState
from dynamixel_hardware_interface.srv import SetVelocity

from std_msgs.msg import Float64

from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_capture.srv import StartAudioRecording
from ua_audio_capture.srv import StopAudioRecording


ACTION_NAME = 'shake_pitch_object'


class ShakePitchObjectActionServer:
    def __init__(self):
        self.get_grasp_status_srv = rospy.ServiceProxy('/wubble_grasp_status', GraspStatus)
        self.start_audio_recording_srv = rospy.ServiceProxy('/audio_dump/start_audio_recording', StartAudioRecording)
        self.stop_audio_recording_srv = rospy.ServiceProxy('/audio_dump/stop_audio_recording', StopAudioRecording)
        self.wrist_pitch_velocity_srv = rospy.ServiceProxy('/wrist_pitch_controller/set_velocity', SetVelocity)
        
        self.wrist_pitch_command_pub = rospy.Publisher('wrist_pitch_controller/command', Float64)
        self.action_server = SimpleActionServer(ACTION_NAME,
                                                ShakePitchObjectAction,
                                                execute_cb=self.shake_pitch_object,
                                                auto_start=False)

    def initialize(self):
        rospy.loginfo('%s: waiting for wubble_grasp_status service' % ACTION_NAME)
        rospy.wait_for_service('/wubble_grasp_status')
        rospy.loginfo('%s: connected to wubble_grasp_status service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for audio_dump/start_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/start_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/start_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for audio_dump/stop_audio_recording service' % ACTION_NAME)
        rospy.wait_for_service('audio_dump/stop_audio_recording')
        rospy.loginfo('%s: connected to audio_dump/stop_audio_recording service' % ACTION_NAME)
        
        rospy.loginfo('%s: waiting for wrist_pitch_controller service' % ACTION_NAME)
        rospy.wait_for_service('/wrist_pitch_controller/set_velocity')
        rospy.loginfo('%s: connected to wrist_pitch_controller service' % ACTION_NAME)
        
        self.action_server.start()

    def shake_pitch_object(self, goal):
        if self.action_server.is_preempt_requested():
            rospy.loginfo('%s: preempted' % ACTION_NAME)
            self.action_server.set_preempted()
            
        wrist_pitch_state = '/wrist_pitch_controller/state'
        desired_velocity = 6.0
        distance = 1.0
        threshold = 0.1
        timeout = 2.0
        
        try:
            msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
            current_pos = msg.position
            start_pos = current_pos
            
            # set wrist to initial position
            self.wrist_pitch_velocity_srv(3.0)
            self.wrist_pitch_command_pub.publish(distance)
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            
            while current_pos < distance-threshold and rospy.Time.now() < end_time:
                msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
                current_pos = msg.position
                rospy.sleep(10e-3)
                
            self.wrist_pitch_velocity_srv(desired_velocity)
            
            # start recording sound and shaking
            self.start_audio_recording_srv(InfomaxAction.SHAKE_PITCH, goal.category_id)
            rospy.sleep(0.5)
            
            for i in range(2):
                self.wrist_pitch_command_pub.publish(-distance)
                end_time = rospy.Time.now() + rospy.Duration(timeout)
                
                while current_pos > -distance+threshold and rospy.Time.now() < end_time:
                    msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
                    current_pos = msg.position
                    rospy.sleep(10e-3)
                    
                self.wrist_pitch_command_pub.publish(distance)
                end_time = rospy.Time.now() + rospy.Duration(timeout)
                
                while current_pos < distance-threshold and rospy.Time.now() < end_time:
                    msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
                    current_pos = msg.position
                    rospy.sleep(10e-3)
                    
            rospy.sleep(0.5)
            
            # check if are still holding an object after shaking
            sound_msg = None
            grasp_status = self.get_grasp_status_srv()
            
            if grasp_status.is_hand_occupied:
                sound_msg = self.stop_audio_recording_srv(True)
            else:
                self.stop_audio_recording_srv(False)
                
            # reset wrist to starting position
            self.wrist_pitch_velocity_srv(3.0)
            self.wrist_pitch_command_pub.publish(start_pos)
            end_time = rospy.Time.now() + rospy.Duration(timeout)
            
            while current_pos < distance-threshold and rospy.Time.now() < end_time:
                msg = rospy.wait_for_message(wrist_pitch_state, DynamixelJointState, timeout)
                current_pos = msg.position
                rospy.sleep(10e-3)
                
            rospy.sleep(0.5)
            
            if sound_msg:
                self.action_server.set_succeeded(ShakePitchObjectResult(sound_msg.recorded_sound))
                return
            else:
                self.action_server.set_aborted()
                return
        except:
            rospy.logerr('%s: attempted pitch failed - %s' % ACTION_NAME)
            self.stop_audio_recording_srv(False)
            self.action_server.set_aborted()


if __name__ == '__main__':
    rospy.init_node('shake_pitch_action_server', anonymous=True)
    spoas = ShakePitchObjectActionServer()
    spoas.initialize()
    rospy.spin()

