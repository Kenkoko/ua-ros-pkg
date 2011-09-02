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

import math
from threading import Thread
from threading import Lock

import roslib; roslib.load_manifest('wubble2_robot')
import rospy

from tf import TransformListener
from tf import LookupException
from tf import ConnectivityException

from dynamixel_controllers.srv import SetSpeed
from dynamixel_controllers.srv import SetTorqueLimit
from dynamixel_msgs.msg import JointState
from phidgets_ros.msg import Float64Stamped

import actionlib
from std_msgs.msg import Float64
from wubble2_robot.msg import WubbleGripperAction
from wubble2_robot.msg import WubbleGripperGoal

#TODO: fix this
DXL_MAX_SPEED_RAD = 6.0

class GripperActionController():
    def __init__(self):
        rospy.init_node('gripper_action_controller', anonymous=True)
        
        self.left_finger_controller = 'left_finger_controller'
        self.right_finger_controller = 'right_finger_controller'
        
        self.left_finger_joint = rospy.get_param(self.left_finger_controller + '/joint_name')
        self.right_finger_joint = rospy.get_param(self.right_finger_controller + '/joint_name')
        
        self.tf_listener = TransformListener()
        self.lr_pub_lock = Lock()   # publish to left and right is atomic
        
        # Publishers and Subscribers for all gripper joint controllers
        self.left_finger_joint_torque_pub = rospy.Publisher(self.left_finger_controller + '/command', Float64)
        self.right_finger_joint_torque_pub = rospy.Publisher(self.right_finger_controller + '/command', Float64)
        
        self.left_finger_joint_state_sub = rospy.Subscriber(self.left_finger_controller + '/state', JointState, self.process_left_finger_joint_state)
        self.right_finger_joint_state_sub = rospy.Subscriber(self.right_finger_controller + '/state', JointState, self.process_right_finger_joint_state)
        
        self.gripper_opening_pub = rospy.Publisher('gripper_opening', Float64)
        self.left_finger_ground_distance_pub = rospy.Publisher('left_finger_ground_distance', Float64)
        self.right_finger_ground_distance_pub = rospy.Publisher('right_finger_ground_distance', Float64)
        
        # Pressure sensors
        # 0-3 - left finger
        # 4-7 - right finger
        num_sensors = 8
        self.pressure = [0.0] * num_sensors
        [rospy.Subscriber('/interface_kit/124427/sensor/%d' % i, Float64Stamped, self.process_pressure_sensors, i) for i in range(num_sensors)]
        
        # pressure sensors are at these values when no external pressure is applied
        self.pressure_left_init = [0.0, 0.0, 0.0, 0.0]
        self.pressure_right_init = [0.0, 0.0, 130.0, 0.0]
        self.pressure_init = self.pressure_left_init + self.pressure_right_init
        self.left_pressure_pub = rospy.Publisher('left_finger_pressure', Float64)
        self.right_pressure_pub = rospy.Publisher('right_finger_pressure', Float64)
        self.total_pressure_pub = rospy.Publisher('total_pressure', Float64)
        
        self.close_gripper = False
        self.overheating = False
        self.dynamic_torque_control = False
        self.lower_pressure = 800.0
        self.upper_pressure = 1000.0
        
        # IR sensor
        self.gripper_ir_pub = rospy.Publisher('gripper_distance_sensor', Float64)
        self.ir_distance = 0.0
        rospy.Subscriber('/interface_kit/106950/sensor/7', Float64Stamped, self.process_ir_sensor)
        
        # Make sure we receive current joint states before starting threads
        rospy.loginfo('Waiting to receive gripper joints state...')
        self.left_finger_joint_state = rospy.wait_for_message(self.left_finger_controller + '/state', JointState)
        self.right_finger_joint_state = rospy.wait_for_message(self.right_finger_controller + '/state', JointState)
        
        # Temperature monitor and torque control thread
        Thread(target=self.gripper_monitor).start()
        
        # Start gripper opening monitoring thread
        Thread(target=self.calculate_gripper_opening).start()
        
        self.action_server = actionlib.SimpleActionServer('wubble_gripper_action',
                                                          WubbleGripperAction,
                                                          execute_cb=self.process_gripper_action,
                                                          auto_start=False)
        self.action_server.start()
        rospy.loginfo('gripper_freespin_controller: ready to accept goals')


    def within_tolerance(self, a, b, tolerance):
        return abs(a - b) < tolerance


    def gripper_monitor(self):
        rospy.loginfo('Gripper temperature monitor and torque control thread started successfully')
        pre_overheat_trigger = False
        overheat_trigger = False
        max_pressure = 8000.0
        r = rospy.Rate(500)
        
        while not rospy.is_shutdown():
            pressure_left = max(0.0, sum(self.pressure[:4]) - sum(self.pressure_left_init))
            pressure_right = max(0.0, sum(self.pressure[4:]) - sum(self.pressure_right_init))
            pressure = pressure_left + pressure_right
            
            self.left_pressure_pub.publish(pressure_left)
            self.right_pressure_pub.publish(pressure_right)
            self.total_pressure_pub.publish(pressure)
            
            #----------------------- TEMPERATURE MONITOR ---------------------------#
            l_temp = self.left_finger_joint_state.motor_temps[0]
            r_temp = self.right_finger_joint_state.motor_temps[0]
            
            l_cur_torque = self.left_finger_joint_state.load * DXL_MAX_SPEED_RAD
            r_cur_torque = self.right_finger_joint_state.load * DXL_MAX_SPEED_RAD
            
            # lower torque to let the motors cool down
            l_goal = -0.2 * DXL_MAX_SPEED_RAD if l_cur_torque < 0.0 else  0.2 * DXL_MAX_SPEED_RAD
            r_goal =  0.2 * DXL_MAX_SPEED_RAD if r_cur_torque > 0.0 else -0.2 * DXL_MAX_SPEED_RAD
            
            if l_temp >= 76 or r_temp >= 76:
                if not overheat_trigger:
                    rospy.logwarn('Disabling gripper motors torque [LM: %dC, RM: %dC]' % (l_temp, r_temp))
                    self.lr_pub_lock.acquire()
                    self.left_finger_joint_torque_pub.publish(-0.5)
                    self.right_finger_joint_torque_pub.publish(0.5)
                    self.lr_pub_lock.release()
                    overheat_trigger = True
            elif l_temp >= 73 or r_temp >= 73:
                if not pre_overheat_trigger and not overheat_trigger:
                    rospy.logwarn('Gripper motors are overheating [LM: %dC, RM: %dC]' % (l_temp, r_temp))
                    rospy.logwarn('Setting torque to [LM: %.2f, RM: %.2f]' % (l_goal, r_goal))
                    self.lr_pub_lock.acquire()
                    self.left_finger_joint_torque_pub.publish(l_goal)
                    self.right_finger_joint_torque_pub.publish(r_goal)
                    self.lr_pub_lock.release()
                    pre_overheat_trigger = True
            else:
                pre_overheat_trigger = False
                overheat_trigger = False
                
            self.overheating = pre_overheat_trigger or overheat_trigger
            #########################################################################
            
            # don't do torque control if not requested or
            # when the gripper is open
            # or when motors are too hot
            if not self.dynamic_torque_control or \
               not self.close_gripper or \
               self.overheating:
                r.sleep()
                continue
                
            #----------------------- TORQUE CONTROL -------------------------------#
            l_current = self.left_finger_joint_state.goal_pos
            r_current = self.right_finger_joint_state.goal_pos
            
            if pressure > self.upper_pressure:   # release
                pressure_change_step = abs(pressure - self.upper_pressure) / max_pressure
                left = min(DXL_MAX_SPEED_RAD, l_current + pressure_change_step)
                right = max(-DXL_MAX_SPEED_RAD, r_current - pressure_change_step)
                
                if left < DXL_MAX_SPEED_RAD or right > -DXL_MAX_SPEED_RAD:
                    if self.close_gripper:
                        self.lr_pub_lock.acquire()
                        self.left_finger_joint_torque_pub.publish(left)
                        self.right_finger_joint_torque_pub.publish(right)
                        self.lr_pub_lock.release()
                        rospy.logdebug('>MAX pressure is %.2f, LT: %.2f, RT: %.2f, step is %.2f' % (pressure, l_current, r_current, pressure_change_step))
            elif pressure < self.lower_pressure: # squeeze
                pressure_change_step = abs(pressure - self.lower_pressure) / max_pressure
                left = max(-DXL_MAX_SPEED_RAD, l_current - pressure_change_step)
                right = min(DXL_MAX_SPEED_RAD, r_current + pressure_change_step)
                
                if left > -DXL_MAX_SPEED_RAD or right < DXL_MAX_SPEED_RAD:
                    if self.close_gripper:
                        self.lr_pub_lock.acquire()
                        self.left_finger_joint_torque_pub.publish(left)
                        self.right_finger_joint_torque_pub.publish(right)
                        self.lr_pub_lock.release()
                        rospy.logdebug('<MIN pressure is %.2f, LT: %.2f, RT: %.2f, step is %.2f' % (pressure, l_current, r_current, pressure_change_step))
            ########################################################################
            
            r.sleep()


    def calculate_gripper_opening(self):
        r = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            try:
                map_frame_id = 'base_footprint'
                gripper_palm_frame_id = 'L7_wrist_roll_link'
                left_fingertip_frame_id = 'left_fingertip_link'
                right_fingertip_frame_id = 'right_fingertip_link'
                
                self.tf_listener.waitForTransform(gripper_palm_frame_id, left_fingertip_frame_id, rospy.Time(0), rospy.Duration(0.2))
                self.tf_listener.waitForTransform(gripper_palm_frame_id, right_fingertip_frame_id, rospy.Time(0), rospy.Duration(0.2))
                
                l_pos, _ = self.tf_listener.lookupTransform(gripper_palm_frame_id, left_fingertip_frame_id, rospy.Time(0))
                r_pos, _ = self.tf_listener.lookupTransform(gripper_palm_frame_id, right_fingertip_frame_id, rospy.Time(0))
                
                dx = l_pos[0] - r_pos[0]
                dy = l_pos[1] - r_pos[1]
                dz = l_pos[2] - r_pos[2]
                
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                self.gripper_opening_pub.publish(dist)
                
                l_pos, _ = self.tf_listener.lookupTransform(map_frame_id, left_fingertip_frame_id, rospy.Time(0))
                r_pos, _ = self.tf_listener.lookupTransform(map_frame_id, right_fingertip_frame_id, rospy.Time(0))
                
                self.left_finger_ground_distance_pub.publish(l_pos[2])
                self.right_finger_ground_distance_pub.publish(r_pos[2])
            except LookupException as le:
                rospy.logerr(le)
            except ConnectivityException as ce:
                rospy.logerr(ce)
            except Exception as e:
                rospy.logerr('Unknown error occured: %s' % str(e))
                
            r.sleep()


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
        if msg.data < 80: self.ir_distance = 1.0           # too far
        elif msg.data > 530: self.ir_distance = 0.0        # too close
        else: self.ir_distance =  20.76 / (msg.data - 11)   # just right
        
        self.gripper_ir_pub.publish(self.ir_distance)


    def process_gripper_action(self, req):
        r = rospy.Rate(500)
        timeout = rospy.Duration(2.0)
        
        if req.command == WubbleGripperGoal.CLOSE_GRIPPER:
            self.dynamic_torque_control = req.dynamic_torque_control
            desired_torque = req.torque_limit * DXL_MAX_SPEED_RAD
            self.lr_pub_lock.acquire()
            self.left_finger_joint_torque_pub.publish(-desired_torque)
            self.right_finger_joint_torque_pub.publish(desired_torque)
            self.lr_pub_lock.release()
            
            self.lower_pressure = req.pressure_lower
            self.upper_pressure = req.pressure_upper
            
            #while self.left_finger_joint_state.is_moving or self.right_finger_joint_state.is_moving:
            if not self.dynamic_torque_control:
                start_time = rospy.Time.now()
                while (not self.within_tolerance(self.left_finger_joint_state.load, -req.torque_limit, 0.01) or
                       not self.within_tolerance(self.right_finger_joint_state.load, -req.torque_limit, 0.01)) and \
                       rospy.Time.now() - start_time < timeout and \
                      not rospy.is_shutdown():
                    r.sleep()
            else:
                if desired_torque > 1e-3: rospy.sleep(1.0 / desired_torque)
                
            self.close_gripper = True
            self.action_server.set_succeeded()
        elif req.command == WubbleGripperGoal.OPEN_GRIPPER:
            self.close_gripper = False
            desired_torque = req.torque_limit * DXL_MAX_SPEED_RAD
            self.lr_pub_lock.acquire()
            self.left_finger_joint_torque_pub.publish(desired_torque)
            self.right_finger_joint_torque_pub.publish(-desired_torque)
            self.lr_pub_lock.release()
            
            start_time = rospy.Time.now()
            while (self.left_finger_joint_state.current_pos < 0.8 or
                   self.right_finger_joint_state.current_pos > -0.8) and \
                   rospy.Time.now() - start_time < timeout and \
                  not rospy.is_shutdown():
                r.sleep()
                
            self.lr_pub_lock.acquire()
            self.left_finger_joint_torque_pub.publish(0.5)
            self.right_finger_joint_torque_pub.publish(-0.5)
            self.lr_pub_lock.release()
            self.action_server.set_succeeded()
        else:
            msg = 'Unrecognized command: %d' % req.command
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)


if __name__ == '__main__':
    try:
        gac = GripperActionController()
        rospy.spin()
    except rospy.ROSInterruptException: pass

