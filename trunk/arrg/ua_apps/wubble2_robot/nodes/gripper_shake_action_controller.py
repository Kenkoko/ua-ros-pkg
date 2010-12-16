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
from wubble2_robot.msg import WubbleGripperShakeAction
from wubble2_robot.msg import WubbleGripperShakeGoal

class GripperActionController():
    def __init__(self):
        rospy.init_node('gripper_action_controller', anonymous=True)
        
        # Wrist stuff
        self.wrist_pitch_controller = 'wrist_pitch_controller'
        self.wrist_yaw_controller = 'wrist_yaw_controller'
        self.wrist_roll_controller = 'wrist_roll_controller'
        
        self.wrist_pitch_joint = rospy.get_param(self.wrist_pitch_controller + '/joint_name')
        self.wrist_yaw_joint = rospy.get_param(self.wrist_yaw_controller + '/joint_name')
        self.wrist_roll_joint = rospy.get_param(self.wrist_roll_controller + '/joint_name')
        
        self.wrist_pitch_joint_state = JointState(name=self.wrist_pitch_joint)
        self.wrist_yaw_joint_state = JointState(name=self.wrist_yaw_joint)
        self.wrist_roll_joint_state = JointState(name=self.wrist_roll_joint)
        
        # Publishers and Subscribers for all gripper joint controllers
        self.wrist_pitch_joint_position_pub = rospy.Publisher(self.wrist_pitch_controller + '/command', Float64)
        self.wrist_yaw_joint_position_pub = rospy.Publisher(self.wrist_yaw_controller + '/command', Float64)
        self.wrist_roll_joint_position_pub = rospy.Publisher(self.wrist_roll_controller + '/command', Float64)
        
        self.wrist_pitch_joint_state_sub = rospy.Subscriber(self.wrist_pitch_controller + '/state', JointState, self.process_wrist_pitch_joint_state)
        self.wrist_yaw_joint_state_sub = rospy.Subscriber(self.wrist_yaw_controller + '/state', JointState, self.process_wrist_yaw_joint_state)
        self.wrist_roll_joint_state_sub = rospy.Subscriber(self.wrist_roll_controller + '/state', JointState, self.process_wrist_roll_joint_state)
        
        # Services
        self.wrist_pitch_joint_velocity_srv = rospy.ServiceProxy(self.wrist_pitch_controller + '/set_speed', SetSpeed)
        self.wrist_yaw_joint_velocity_srv = rospy.ServiceProxy(self.wrist_yaw_controller + '/set_speed', SetSpeed)
        self.wrist_roll_joint_velocity_srv = rospy.ServiceProxy(self.wrist_roll_controller + '/set_speed', SetSpeed)
        
        self.wrist_pitch_joint_torque_limit_srv = rospy.ServiceProxy(self.wrist_pitch_controller + '/set_torque_limit', SetTorqueLimit)
        self.wrist_yaw_joint_torque_limit_srv = rospy.ServiceProxy(self.wrist_yaw_controller + '/set_torque_limit', SetTorqueLimit)
        self.wrist_roll_joint_torque_limit_srv = rospy.ServiceProxy(self.wrist_roll_controller + '/set_torque_limit', SetTorqueLimit)
        
        self.wrist_pitch_joint_slope_srv = rospy.ServiceProxy(self.wrist_pitch_controller + '/set_compliance_slope', SetComplianceSlope)
        self.wrist_yaw_joint_slope_srv = rospy.ServiceProxy(self.wrist_yaw_controller + '/set_compliance_slope', SetComplianceSlope)
        self.wrist_roll_joint_slope_srv = rospy.ServiceProxy(self.wrist_roll_controller + '/set_compliance_slope', SetComplianceSlope)
        
        # Grpper stuff
        self.left_finger_controller = 'gripper_left_finger_controller'
        self.right_finger_controller = 'gripper_right_finger_controller'
        
        self.left_finger_joint = rospy.get_param(self.left_finger_controller + '/joint_name')
        self.right_finger_joint = rospy.get_param(self.right_finger_controller + '/joint_name')
        
        self.left_finger_joint_state = JointState(name=self.left_finger_joint)
        self.right_finger_joint_state = JointState(name=self.right_finger_joint)
        
        # Publishers and Subscribers for all gripper joint controllers
        self.left_finger_joint_position_pub = rospy.Publisher(self.left_finger_controller + '/command', Float64)
        self.right_finger_joint_position_pub = rospy.Publisher(self.right_finger_controller + '/command', Float64)
        
        self.left_finger_joint_state_sub = rospy.Subscriber(self.left_finger_controller + '/state', JointState, self.process_left_finger_joint_state)
        self.right_finger_joint_state_sub = rospy.Subscriber(self.right_finger_controller + '/state', JointState, self.process_right_finger_joint_state)
        
        # And finally start up the action server
        self.action_server = actionlib.SimpleActionServer('wubble_gripper_shake_action', WubbleGripperShakeAction, execute_cb=self.process_gripper_shake_action)
        rospy.loginfo('gripper_action_controller: ready to accept goals')

    def __copy_state(self, state, msg):
        state.goal_pos = msg.goal_pos
        state.current_pos = msg.current_pos
        state.error = msg.error
        state.velocity = msg.velocity
        state.load = msg.load
        state.is_moving = msg.is_moving

    def process_wrist_pitch_joint_state(self, msg):
        self.__copy_state(self.wrist_pitch_joint_state, msg)

    def process_wrist_yaw_joint_state(self, msg):
        self.__copy_state(self.wrist_yaw_joint_state, msg)

    def process_wrist_roll_joint_state(self, msg):
        self.__copy_state(self.wrist_roll_joint_state, msg)

    def process_left_finger_joint_state(self, msg):
        self.__copy_state(self.left_finger_joint_state, msg)

    def process_right_finger_joint_state(self, msg):
        self.__copy_state(self.right_finger_joint_state, msg)

    def process_gripper_shake_action(self, req):
        desired_velocity = 3.0
        distance = 0.5
        self.wrist_yaw_joint_velocity_srv(desired_velocity)
        
        ############## SHAKE SIDE2SIDE ################################
        # set current position as goal so motors won't overload, maybe?
        lf_old_state = self.left_finger_joint_state.current_pos
        rf_old_state = self.right_finger_joint_state.current_pos
        
        self.left_finger_joint_position_pub.publish(lf_old_state - 0.2)
        self.right_finger_joint_position_pub.publish(rf_old_state + 0.2)
        
        for i in range(req.shake_number):
            self.wrist_yaw_joint_position_pub.publish(-distance)
            rospy.sleep(distance/desired_velocity + 0.05)
            self.wrist_yaw_joint_position_pub.publish(distance)
            rospy.sleep(distance/desired_velocity + 0.05)
            
        self.wrist_yaw_joint_velocity_srv(2.0)
        self.wrist_yaw_joint_position_pub.publish(0.0)
        self.left_finger_joint_position_pub.publish(lf_old_state - 0.01)
        self.right_finger_joint_position_pub.publish(rf_old_state + 0.01)
        
        ############# SHAKE ROLLEMSIDE2SIDE ############################
        desired_velocity = 4.0
        distance = 1.0
        self.wrist_roll_joint_velocity_srv(desired_velocity)
        
        # set current position as goal so motors won't overload, maybe?
        roll_old_state = self.wrist_roll_joint_state.current_pos
        lf_old_state = self.left_finger_joint_state.current_pos
        rf_old_state = self.right_finger_joint_state.current_pos
        
        self.left_finger_joint_position_pub.publish(lf_old_state - 0.2)
        self.right_finger_joint_position_pub.publish(rf_old_state + 0.2)
        
        for i in range(req.shake_number):
            self.wrist_roll_joint_position_pub.publish(roll_old_state - distance)
            rospy.sleep(distance/desired_velocity + 0.05)
            self.wrist_roll_joint_position_pub.publish(roll_old_state + distance)
            rospy.sleep(distance/desired_velocity + 0.05)
            
        self.wrist_roll_joint_velocity_srv(2.0)
        self.wrist_roll_joint_position_pub.publish(roll_old_state)
        self.left_finger_joint_position_pub.publish(lf_old_state - 0.01)
        self.right_finger_joint_position_pub.publish(rf_old_state + 0.01)
        
        ############## SHAKE SIDE2SIDE ################################
        desired_velocity = 3.0
        distance = 0.5
        self.wrist_pitch_joint_velocity_srv(desired_velocity)
        
        # set current position as goal so motors won't overload, maybe?
        lf_old_state = self.left_finger_joint_state.current_pos
        rf_old_state = self.right_finger_joint_state.current_pos
        
        self.left_finger_joint_position_pub.publish(lf_old_state - 0.2)
        self.right_finger_joint_position_pub.publish(rf_old_state + 0.2)
        
        for i in range(req.shake_number):
            self.wrist_pitch_joint_position_pub.publish(-distance)
            rospy.sleep(distance/desired_velocity + 0.05)
            self.wrist_pitch_joint_position_pub.publish(distance)
            rospy.sleep(distance/desired_velocity + 0.05)
            
        self.wrist_pitch_joint_velocity_srv(2.0)
        self.wrist_pitch_joint_position_pub.publish(0.0)
        self.left_finger_joint_position_pub.publish(lf_old_state - 0.01)
        self.right_finger_joint_position_pub.publish(rf_old_state + 0.01)
        
        self.action_server.set_succeeded()

if __name__ == '__main__':
    gac = GripperActionController()
    rospy.spin()

