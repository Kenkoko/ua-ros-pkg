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
from ua_controller_msgs.msg import JointState

import actionlib
from std_msgs.msg import Float64
from wubble2_robot.msg import WubbleGripperAction
from wubble2_robot.msg import WubbleGripperGoal

class GripperActionController():
    def __init__(self):
        rospy.init_node('gripper_action_controller', anonymous=True)
        
        self.left_finger_controller = rospy.get_param('~left_finger_controller')
        self.right_finger_controller = rospy.get_param('~right_finger_controller')
        
        self.left_finger_joint = rospy.get_param(self.left_finger_controller + '/joint_name')
        self.right_finger_joint = rospy.get_param(self.right_finger_controller + '/joint_name')
        
        self.left_finger_joint_state = JointState(name=self.left_finger_joint)
        self.right_finger_joint_state = JointState(name=self.right_finger_joint)
        
        # Publishers and Subscribers for all gripper joint controllers
        self.left_finger_joint_position_pub = rospy.Publisher(self.left_finger_controller + '/command', Float64)
        self.right_finger_joint_position_pub = rospy.Publisher(self.right_finger_controller + '/command', Float64)
        
        self.left_finger_joint_state_sub = rospy.Subscriber(self.left_finger_controller + '/state', JointState, self.process_left_finger_joint_state)
        self.right_finger_joint_state_sub = rospy.Subscriber(self.right_finger_controller + '/state', JointState, self.process_right_finger_joint_state)
        
        # Services
        self.left_finger_joint_velocity_srv = rospy.ServiceProxy(self.left_finger_controller + '/set_speed', SetSpeed)
        self.right_finger_joint_velocity_srv = rospy.ServiceProxy(self.right_finger_controller + '/set_speed', SetSpeed)
        self.left_finger_joint_torque_limit_srv = rospy.ServiceProxy(self.left_finger_controller + '/set_torque_limit', SetTorqueLimit)
        self.right_finger_joint_torque_limit_srv = rospy.ServiceProxy(self.right_finger_controller + '/set_torque_limit', SetTorqueLimit)
        
        self.action_server = actionlib.SimpleActionServer('wubble_gripper_action', WubbleGripperAction, execute_cb=self.process_gripper_action)
        
        rospy.loginfo('gripper_action_controller: ready to accept goals')

    def __copy_state(self, state, msg):
        state.goal_pos = msg.goal_pos
        state.current_pos = msg.current_pos
        state.error = msg.error
        state.velocity = msg.velocity
        state.load = msg.load
        state.is_moving = msg.is_moving

    def process_left_finger_joint_state(self, msg):
        self.__copy_state(self.left_finger_joint_state, msg)

    def process_right_finger_joint_state(self, msg):
        self.__copy_state(self.right_finger_joint_state, msg)

    def process_gripper_action(self, req):
        if req.command == WubbleGripperGoal.CLOSE_GRIPPER:
            self.left_finger_joint_torque_limit_srv(req.torque_limit)
            self.right_finger_joint_torque_limit_srv(req.torque_limit)
            self.left_finger_joint_position_pub.publish(0.0)
            self.right_finger_joint_position_pub.publish(0.0)
            rospy.sleep(0.1)
            
            r = rospy.Rate(500)
            while self.left_finger_joint_state.is_moving or self.right_finger_joint_state.is_moving:
                r.sleep()
                
            # set current position as goal so motors won't overload, maybe?
            self.left_finger_joint_position_pub.publish(self.left_finger_joint_state.current_pos - 0.05)
            self.right_finger_joint_position_pub.publish(self.right_finger_joint_state.current_pos + 0.05)
            rospy.loginfo('CLosed gripper')
            
            self.action_server.set_succeeded()
        elif req.command == WubbleGripperGoal.OPEN_GRIPPER:
            self.left_finger_joint_torque_limit_srv(req.torque_limit)
            self.right_finger_joint_torque_limit_srv(req.torque_limit)
            self.left_finger_joint_position_pub.publish(1.0)
            self.right_finger_joint_position_pub.publish(-1.0)
            rospy.loginfo('Opned gripper')
            self.action_server.set_succeeded()
        else:
            rospy.logerr('Unrecognized command, gripper is not moving enywhere!')
            return

if __name__ == '__main__':
    gac = GripperActionController()
    rospy.spin()

