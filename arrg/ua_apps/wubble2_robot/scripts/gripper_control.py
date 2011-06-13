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
        self.gripper_client = SimpleActionClient('wubble_gripper_action', WubbleGripperAction)
        self.gripper_client.wait_for_server()

    def close_gripper(self, torque_limit, dynamic_torque_control):
        goal = WubbleGripperGoal()
        goal.command = WubbleGripperGoal.CLOSE_GRIPPER
        goal.torque_limit = torque_limit
        goal.dynamic_torque_control = dynamic_torque_control
        goal.pressure_upper = 1500.0
        goal.pressure_lower = 1300.0
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def open_gripper(self):
        goal = WubbleGripperGoal()
        goal.command = WubbleGripperGoal.OPEN_GRIPPER
        goal.torque_limit = 0.4
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        osc = ObjectSoundCollector()
        rospy.sleep(2)
        rospy.loginfo('Closing gripper')
        #osc.close_gripper(1.0, False)
        osc.close_gripper(0.05, False)
        #osc.open_gripper()
    except rospy.ROSInterruptException:
        pass

