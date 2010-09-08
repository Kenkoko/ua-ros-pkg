#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
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

# Author: Tom Walsh (forked from Anh Tran)

PKG = 'block_swat'
NAME = 'actions_demo'

import roslib; roslib.load_manifest(PKG)
import rospy
import re
from math import atan2

#from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import * 
from wubble_actions.msg import *
from actionlib import SimpleActionClient
#from smart_arm_kinematics.srv import SmartArmIK
#from std_msgs.msg import Float64
#from geometry_msgs.msg import PointStamped
from pr2_controllers_msgs.msg import JointControllerState
from gazebo.msg  import ModelStates
from tf.transformations import quaternion_from_euler


import time

class Swatter():

	def get_sp(self):
		return self.shoulder_pan.process_value
	def get_st(self):
		return self.shoulder_tilt.process_value
	def get_et(self):
		return self.elbow_tilt.process_value
	def get_wr(self):
		return self.wrist_rotate.process_value

	def printJoints(self):
		print str(self.get_sp()) + " ,  " + str(self.get_st()) + " , " + str(self.get_et()) + " , " + str(self.get_wr()) 

	def get_blocks(self):
		allBlocks = []
		sequence = self.model_data.name
		for i, n in  zip(range(len(sequence)), sequence):
			if re.search('_swat_', n):
				allBlocks.append(self.model_data.pose[i])
		return allBlocks

	def __init__(self):
		self.shoulder_tilt = 0.0
		self.shoulder_pan =0.0
		self.elbow_tilt = 0.0
		self.wrist_rotate = 0.0
		self.model_data = None
		self.base_angle = 0.0
		#print "Now subscribe"
		rospy.Subscriber('shoulder_tilt_controller/state', JointControllerState, self.read_shoulder_tilt)
		#rospy.wait_for_message('head_pan_controller/state', JointControllerState)
		#print "did a subscription"
		rospy.Subscriber('shoulder_pan_controller/state', JointControllerState, self.read_shoulder_pan)
		#rospy.wait_for_message('head_pan_controller/state', JointControllerState)
		rospy.Subscriber('elbow_tilt_controller/state', JointControllerState, self.read_elbow_tilt)
		#rospy.wait_for_message('head_pan_controller/state', JointControllerState)
		rospy.Subscriber('wrist_rotate_controller/state', JointControllerState, self.read_wrist_rotate)
		#rospy.wait_for_message('head_pan_controller/state', JointControllerState)
		rospy.Subscriber('gazebo/model_states', ModelStates, self.read_model)
		#rospy.Subscriber

	def read_shoulder_tilt(self, data):
		self.shoulder_tilt = data

	def read_shoulder_pan(self, data):
		self.shoulder_pan = data

	def read_elbow_tilt(self, data):
		self.elbow_tilt = data
	def read_wrist_rotate(self, data):
		self.wrist_rotate = data
	
	def read_model(self, data):
		self.model_data = data



def move_head(head_pan, head_tilt):
	goal = WubbleHeadGoal()
	goal.target_joints = [head_pan, head_tilt]
	head_client.send_goal(goal)
	head_client.wait_for_result()
	result = head_client.get_result()
	if result.success == False:
		print "Action failed"
	else:
		print "Result: [" + str(result.head_position[0]) + ", " + str(result.head_position[1]) + "]"


def look_at(frame_id, x, y, z):
	goal = WubbleHeadGoal()
	goal.target_point = PointStamped()
	goal.target_point.header.frame_id = frame_id
	goal.target_point.point.x = x
	goal.target_point.point.y = y
	goal.target_point.point.z = z
	head_client.send_goal(goal)
	head_client.wait_for_result()
	result = head_client.get_result()
	if result.success == False:
		print "Action failed"
	else:
		print "Result: [" + str(result.head_position[0]) + ", " + str(result.head_position[1]) + "]"


def move_arm(shoulder_pan, shoulder_tilt, elbow_tilt, wrist_rotate):
	goal = SmartArmGoal()
	goal.target_joints = [shoulder_pan, shoulder_tilt, elbow_tilt, wrist_rotate]
	arm_client.send_goal(goal)
	arm_client.wait_for_result()
	result = arm_client.get_result()
	if result.success == False:
		print "Action failed"
	else:
		print "Result: [" + str(result.arm_position[0]) + ", " + str(result.arm_position[1]) + \
		str(result.arm_position[2]) + ", " + str(result.arm_position[3]) + "]"


def reach_at(frame_id, x, y, z):
	goal = SmartArmGoal()
	goal.target_point = PointStamped()
	goal.target_point.header.frame_id = frame_id
	goal.target_point.point.x = x
	goal.target_point.point.y = y
	goal.target_point.point.z = z
	arm_client.send_goal(goal)
	arm_client.wait_for_result()

	result = arm_client.get_result()
	if result.success == False:
		print "Action failed"
	else:
		print "Result: [" + str(result.arm_position[0]) + ", " + str(result.arm_position[1]) + \
		", " + str(result.arm_position[2]) + ", " + str(result.arm_position[3]) + "]"

def reach_down():

	goal = SmartArmGoal()
        goal.target_joints = [-0.2 ,  -0.200938880444 , -1.21945416927 , -0.000813533843029]
        arm_client.send_goal(goal)
        arm_client.wait_for_result()
        result = arm_client.get_result()
	
	if (result.success == False):
                print "Action failed"
        else:
                print "Reached Down"



def swat(x):
	#assume it is hovering 0.1m over the floor above the center of the object
	
	#get the current joint angles
	orig_sp = x.get_sp()
        orig_st = x.get_st()
        orig_et  =x.get_et()
        orig_wr = x.get_wr()

	#turn shoulder back
	goal = SmartArmGoal()
        goal.target_joints = [orig_sp-0.1, orig_st, orig_et, orig_wr]
        arm_client.send_goal(goal)
        arm_client.wait_for_result()
        result = arm_client.get_result()


	#put the hand on the floor	
	#reach_at("/arm_left_finger_link",0.0,0.0,-0.1)

	#swat
	goal = SmartArmGoal()
	goal.target_joints = [orig_sp+0.4, orig_st, orig_et, orig_wr]

	arm_client.send_goal(goal)
	arm_client.wait_for_result()
	result3 = arm_client.get_result()
	# goal.target_joints = [orig_sp, orig_st, orig_et, orig_wr]

	#arm_client.send_goal(goal)
	#arm_client.wait_for_result()

	#result2 = arm_client.get_result()
    
	if (result.success == False)  or (result3.success == False):
		print "Action failed"
	else:
		print "SWATTED!" 

def move_to(frame_id, position, orientation, vicinity=0.0):
    goal = ErraticBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.pose.position = position
    goal.target_pose.pose.orientation = orientation
    goal.vicinity_range = vicinity

    base_client.send_goal(goal)
    rospy.loginfo("Before wait")
    base_client.wait_for_result()
    rospy.loginfo("After wait")

    if base_client.get_state() == GoalStatus.SUCCEEDED:
        result = base_client.get_result()
        print "Result: " + str(result.base_position)
    elif base_client.get_state() == GoalStatus.PREEMPTED:
        print "Action pre-empted"
    else:
        print "Action failed"


def turn_by(ori):
	position = Point(x =0, y = 0)
	move_to('/base_footprint', position, ori)

def step_back():
    position = Point(x=-0.35, y=0.0)
    orientation = Quaternion(w=1.0)
    move_to('/base_footprint', position, orientation) # No vicinity specified
    position = Point(x=0.67, y=0.0)
    orientation = Quaternion(w=1.0)
    move_to('/base_footprint', position, orientation)


def move_gripper(left_finger, right_finger):
    goal = SmartArmGripperGoal()
    goal.target_joints = [left_finger, right_finger]

    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()

    result = gripper_client.get_result()
    if result.success == False:
        print "Action failed"
    else:
        print "Result: [" + str(result.gripper_position[0]) + ", " + str(result.gripper_position[1]) + "]"


def tilt_laser(n=1):
    goal = HokuyoLaserTiltGoal()
    goal.tilt_cycles = n

    laser_client.send_goal(goal)
    laser_client.wait_for_result()

    result = laser_client.get_result()
    if result.success == False:
        print "Action failed"
    else:
        print "Result: " + str(result.tilt_position)



if __name__ == '__main__':
    try:
	x = Swatter()
	print "Setup Swatter"    
        rospy.init_node(NAME, anonymous=True)
        head_client = SimpleActionClient("wubble_head_action", WubbleHeadAction)
        arm_client = SimpleActionClient("smart_arm_action", SmartArmAction)
        #arm_cient_angled = SimpleActionClient("", )
        gripper_client = SimpleActionClient("smart_arm_gripper_action", SmartArmGripperAction)
        laser_client = SimpleActionClient('hokuyo_laser_tilt_action', HokuyoLaserTiltAction)
        base_client = SimpleActionClient("erratic_base_action", ErraticBaseAction)
        
        base_client.wait_for_server()
        head_client.wait_for_server()
        arm_client.wait_for_server()
        gripper_client.wait_for_server()
        laser_client.wait_for_server()

	

        print "Starting block swatting."

	print "Close gripper"
        move_gripper(-0.15, 0.15);
        time.sleep(1.0)

	

	theBlocks = x.get_blocks()
	#assume it starts facing a block
	lastx = theBlocks[0].position.x
	lasty = theBlocks[0].position.y

	for block in theBlocks:
		print "Going after Object at " + str(block.position.x)  + " " + str(block.position.y) 
		bx = block.position.x
		by = block.position.y
	
		#calculate the new angle (the two points are on a circle)
		if((bx != lastx) and (by != lasty)):
			ori = Quaternion()
			angle = atan2(by , bx) #atan((by - lasty)/(bx - lastx))
			(ori.x, ori.y, ori.z, ori.w) = quaternion_from_euler(0, 0, angle)	
			#turn the robot
			#convert the radians to whatever
			turn_by(ori)

		lastx = bx
		lasty = by		
       # print "Laser tilt"
       # tilt_laser(2)
      #  print "Look at blocks"
      #  look_at("/arm_base_link", 0.2825, 0.0, -0.025)
        #print "Step back"
        #step_back()
        #rospy.sleep(2.0)
		print "Reaching at block"
		#reach_at("/map", bx, by, 0.15)
		reach_down()		

		#really should be over it, backswing and down

        #print "Reach middle stack"
        #reach_at("/arm_base_link", 0.28, -0.0, -0.040)
        #reach_at("/arm_base_link", 0.26, 0.05, -0.060)
        	rospy.sleep(1.0)
		x.printJoints()	
	
		print "SWAT!"
		swat(x)
		rospy.sleep(1.5)
         
        #print "Close gripper"
        #move_gripper(-0.075, 0.075)
        #print "Raise arm"
        #move_arm(0.0, 0.75, -1.972222, 0.0)
	
		print "Reset arm"
        	move_arm(0.0, 1.972222, -1.972222, 0.0)
		rospy.sleep(1.0)

        #print "Reach left stack"
        #reach_at("/arm_base_link", 0.12, 0.260, -0.060)  #was .205
        #rospy.sleep(1.0)
	#print "SWAT!"
	#swat(x)
	#rospy.sleep(1.0)
        

	#print "Open gripper"
        #move_gripper(0.2, -0.2)
        #print "Reset arm"
        #move_arm(0.0, 1.972222, -1.972222, 0.0)

        print "Block swatting completed."

    except rospy.ROSInterruptException:
        pass

