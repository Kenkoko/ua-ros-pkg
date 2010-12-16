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

from ax12_controller_core.srv import SetSpeed
from ax12_controller_core.srv import SetTorqueLimit
from ax12_controller_core.srv import SetComplianceSlope

from ua_controller_msgs.msg import JointState

import actionlib
from std_msgs.msg import Float64
from wubble2_robot.msg import WubbleGripperTapAction
from wubble2_robot.msg import WubbleGripperTapGoal

class GripperActionController():
    def __init__(self):
        rospy.init_node('gripper_tap_action_controller', anonymous=True)
        
        # Wrist stuff
        self.wrist_yaw_controller = 'wrist_yaw_controller'
        self.wrist_yaw_joint = rospy.get_param(self.wrist_yaw_controller + '/joint_name')
        self.wrist_yaw_joint_state = JointState(name=self.wrist_yaw_joint)
        
        # Publishers and Subscribers for all gripper joint controllers
        self.wrist_yaw_joint_position_pub = rospy.Publisher(self.wrist_yaw_controller + '/command', Float64)
        self.wrist_yaw_joint_state_sub = rospy.Subscriber(self.wrist_yaw_controller + '/state', JointState, self.process_wrist_yaw_joint_state)
        
        # Services
        self.wrist_yaw_joint_velocity_srv = rospy.ServiceProxy(self.wrist_yaw_controller + '/set_speed', SetSpeed)
        self.wrist_yaw_joint_torque_limit_srv = rospy.ServiceProxy(self.wrist_yaw_controller + '/set_torque_limit', SetTorqueLimit)
        self.wrist_yaw_joint_slope_srv = rospy.ServiceProxy(self.wrist_yaw_controller + '/set_compliance_slope', SetComplianceSlope)
        
        self.action_server = actionlib.SimpleActionServer('wubble_gripper_tap_action', WubbleGripperTapAction, execute_cb=self.process_gripper_tap_action)
        
        rospy.loginfo('gripper_action_controller: ready to accept goals')

    def __copy_state(self, state, msg):
        state.goal_pos = msg.goal_pos
        state.current_pos = msg.current_pos
        state.error = msg.error
        state.velocity = msg.velocity
        state.load = msg.load
        state.is_moving = msg.is_moving

    def process_wrist_yaw_joint_state(self, msg):
        self.__copy_state(self.wrist_yaw_joint_state, msg)

    def process_gripper_tap_action(self, req):
        desired_velocity = 6.0
        distance = 1
        self.wrist_yaw_joint_velocity_srv(desired_velocity)
        self.wrist_yaw_joint_slope_srv(128)
        
        if req.side == WubbleGripperTapGoal.LEFT_TAP:
            self.wrist_yaw_joint_position_pub.publish(-distance)
            rospy.sleep(distance/desired_velocity + 0.05)
        elif req.side == WubbleGripperTapGoal.RIGHT_TAP:
            self.wrist_yaw_joint_position_pub.publish(distance)
            rospy.sleep(distance/desired_velocity + 0.05)
            
        self.wrist_yaw_joint_velocity_srv(2.0)
        self.wrist_yaw_joint_position_pub.publish(0.0)
        self.wrist_yaw_joint_slope_srv(32)
        self.action_server.set_succeeded()

if __name__ == '__main__':
    gac = GripperActionController()
    rospy.spin()

