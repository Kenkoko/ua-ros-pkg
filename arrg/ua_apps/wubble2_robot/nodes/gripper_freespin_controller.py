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
from phidgets_ros.msg import Float64Stamped

import actionlib
from std_msgs.msg import Float64
from wubble2_robot.msg import WubbleGripperAction
from wubble2_robot.msg import WubbleGripperGoal

class GripperActionController():
    def __init__(self):
        rospy.init_node('gripper_action_controller', anonymous=True)
        
        self.left_finger_controller = 'gripper_left_finger_controller'
        self.right_finger_controller = 'gripper_right_finger_controller'
        
        self.left_finger_joint = rospy.get_param(self.left_finger_controller + '/joint_name')
        self.right_finger_joint = rospy.get_param(self.right_finger_controller + '/joint_name')
        
        self.left_finger_joint_state = None
        self.right_finger_joint_state = None
        
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
        
        # Pressure sensors
        # 0-3 - left finger
        # 4-7 - right finger
        num_sensors = 8
        [rospy.Subscriber('/interface_kit/124427/sensor/%d' % i, Float64Stamped, self.process_pressure_sensors, i) for i in range(num_sensors)]
        self.pressure = [0.0] * num_sensors
        
        # IR sensor
        rospy.Subscriber('/interface_kit/106950/sensor/7', Float64Stamped, self.process_ir_sensor)
        self.ir_distance = 0.0
        
        self.action_server = actionlib.SimpleActionServer('wubble_gripper_action', WubbleGripperAction, execute_cb=self.process_gripper_action)
        
        rospy.loginfo('gripper_freespin_controller: ready to accept goals')

    def process_left_finger_joint_state(self, msg):
        self.left_finger_joint_state = msg

    def process_right_finger_joint_state(self, msg):
        self.right_finger_joint_state = msg

    def process_pressure_sensors(self, msg, i):
        self.pressure[i] = msg.data

    def process_ir_sensor(self, msg):
        """
        Given a raw sensor value from Sharp IR sensor (4-30cm model)
        returns an actual distance in meters.
        """
        if msg.data >= 80: self.ir_distance = 1.0           # too far
        elif msg.data <= 530: self.ir_distance = 0.0        # too close
        else: self.ir_distance =  20.76 / (msg.data - 11)   # just right

    def process_gripper_action(self, req):
        r = rospy.Rate(500)
        
        if req.command == WubbleGripperGoal.CLOSE_GRIPPER:
            self.left_finger_joint_position_pub.publish(-4.0)
            self.right_finger_joint_position_pub.publish(4.0)
            rospy.sleep(0.1)
            
            while self.left_finger_joint_state.is_moving or self.right_finger_joint_state.is_moving:
                r.sleep()
                
            self.left_finger_joint_position_pub.publish(self.left_finger_joint_state.current_pos - 10.0)
            self.right_finger_joint_position_pub.publish(self.right_finger_joint_state.current_pos + 10.0)
            
            self.action_server.set_succeeded()
        elif req.command == WubbleGripperGoal.OPEN_GRIPPER:
            self.left_finger_joint_torque_limit_srv(req.torque_limit)
            self.right_finger_joint_torque_limit_srv(req.torque_limit)
            self.left_finger_joint_position_pub.publish(4.0)
            self.right_finger_joint_position_pub.publish(-4.0)
            
            while self.left_finger_joint_state.current_pos < 0.8 or self.right_finger_joint_state.current_pos > -0.8:
                r.sleep()
                
            self.left_finger_joint_position_pub.publish(0.0)
            self.right_finger_joint_position_pub.publish(0.0)
            self.action_server.set_succeeded()
        else:
            rospy.logerr('Unrecognized command, gripper is not moving enywhere!')
            return

if __name__ == '__main__':
    gac = GripperActionController()
    rospy.spin()

