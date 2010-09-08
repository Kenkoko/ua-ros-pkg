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
#
# Authors: Antons Rebguns
#          Tom Walsh
#

PKG = 'block_swat'

import roslib; roslib.load_manifest(PKG) #was wubble_blocks
import rospy
import re

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


from wubble_actions.msg import ErraticBaseGoal
from wubble_actions.msg import ErraticBaseAction
from wubble_actions.msg import SmartArmGoal
from wubble_actions.msg import SmartArmGripperGoal
from wubble_actions.msg import SmartArmAction
from wubble_actions.msg import SmartArmGripperAction
from gazebo.msg  import ModelStates


from ax12_controller_core.srv import SetSpeed

class ObjectSwat:
    def __init__(self, file):
	self.fileOut = file
	rospy.init_node('object_swatter', anonymous=True)
        self.all_objects = []
        self.all_object_names = []
	self.current_obj_idx = -1
        self.swat_time = rospy.Time.now()

	self.model_data = None
	self.blockToSwat = ""
	self.min_front_dist = 10.0 
	self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
	self.allPoints = []
	self.inProgress = False

    #    rospy.Subscriber('overhead_objects', PoseArray, self.update_object_positions)
       
        rospy.Subscriber('gazebo/model_states', ModelStates, self.read_model) 
        #self.sh_pan_speed_srv = rospy.ServiceProxy('shoulder_pan_controller/set_speed', SetSpeed)
        self.base_client = SimpleActionClient('erratic_base_action', ErraticBaseAction)
        self.arm_client = SimpleActionClient('smart_arm_action', SmartArmAction)
        self.gripper_client = SimpleActionClient('smart_arm_gripper_action', SmartArmGripperAction)
    
	rospy.Subscriber('tilt_laser/scan', LaserScan, self.filter_scan)

    
        #rospy.wait_for_service('shoulder_pan_controller/set_speed')
        self.base_client.wait_for_server()
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        rospy.loginfo('Connected to all action servers')
 
    def setBlockToSwat(self, name):
	self.blockToSwat = name
	if(name != ""):
            self.allPoints = []
            self.inProgress = False    
    
    def read_model(self, data):
        self.model_data = data
        self.all_objects = [];
	self.all_object_names = []
        sequence = self.model_data.name
	for i, n in  zip(range(len(sequence)), sequence):
            if re.search('_swat_', n):
                self.all_objects.append(self.model_data.pose[i])       
		self.all_object_names.append(n)
                if not self.inProgress:
	            return
		if re.search(n, self.blockToSwat):
                    t = rospy.Time.now()
                    if len(self.allPoints) == 0 or t.secs > (self.allPoints[-1])['time1'] or  t.nsecs > (self.allPoints[-1])['time2'] + .05:
                        xx = self.model_data.pose[i].position.x
                        yy = self.model_data.pose[i].position.y
		        self.allPoints.append({'time1': t.secs, 'time2': t.nsecs,  'x': xx, 'y': yy})	
                        if(self.fileOut != None):
		            self.fileOut.write(str(t.secs) + "." +  str(t.nsecs) +  "," + str(xx) + "," +  str(yy) + "\n")
#self.all_objects = msg.poses
        
    def getPoints(self):
	return self.allPoints 

    def tuck_arm_for_navigation(self):
        goal = SmartArmGripperGoal()
        goal.target_joints = [0.15, -0.15]
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        result = self.gripper_client.get_result()
        if result.success: print "Gripper in position"
        else: print "Gripper positioning failed"
        
        goal = SmartArmGoal()
        goal.target_joints = [0.0, 1.77, 0.0, 0.0]
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result()
        result = self.arm_client.get_result()
        if result.success: print "Arm tucked"; return True
        else: print "Failed to tuck arm"; return False
        

    def go_to_object(self, idx):
        self.current_obj_idx = idx

        goal = ErraticBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.pose = self.all_objects[idx]
        goal.vicinity_range = 0.3

        self.base_client.send_goal(goal)
        self.base_client.wait_for_result()

        if self.base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Robot is in front of the object')
            return True
        elif self.base_client.get_state() == GoalStatus.PREEMPTED:
            rospy.loginfo('Action pre-empted')
            return False
        else:
            rospy.loginfo('Action failed')
            return False

    def position_self(self):
        max_speed = 0.1
        cmd_vel = Twist()

        rospy.loginfo('LIDAR reports object is %f m away...', self.min_front_dist)

#	rospy.loginfo('HERE!!!!!!!!!!!!')
        # move in closer
        if (self.min_front_dist > 0.24 and self.min_front_dist <= 0.50):
            range = abs(self.min_front_dist - 0.24)
            rospy.loginfo('Object is too far away, closing in %f m', range)
            cmd_vel.linear.x = max_speed
            cmd_vel.angular.z = 0.0

            r = rospy.Rate(15)
            et = rospy.Time.now() + rospy.Duration.from_sec(range / max_speed)

            while rospy.Time.now() < et:
                self.cmd_vel_pub.publish(cmd_vel)
                r.sleep()

            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
        # back away
        elif self.min_front_dist < 0.22:  #was 0.22
            range = abs(self.min_front_dist - 0.22)
            rospy.loginfo('Object is too close, backing away %f m', range)
            cmd_vel.linear.x = -max_speed
            cmd_vel.angular.z = 0.0

            r = rospy.Rate(15)
            et = rospy.Time.now() + rospy.Duration.from_sec(range / max_speed)

            while rospy.Time.now() < et:
                self.cmd_vel_pub.publish(cmd_vel)
                r.sleep()

            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
        # quit, navigation failed?
        elif self.min_front_dist > 0.50:
            rospy.logwarn('Object is way out there, navigation should take care of traversing great distances')
        else:
            rospy.loginfo('Object is within reach, no need to move')

    def filter_scan(self, scan):
        center_scan_index = (len(scan.ranges) + 1) / 2

        min_idx = center_scan_index - 50
        max_idx = center_scan_index + 50

        for dist in scan.ranges[min_idx:max_idx]:
            if dist < self.min_front_dist and dist > 0.05:
                self.min_front_dist = dist


 
    def swat_object(self):
        goal = SmartArmGripperGoal()
        goal.target_joints = [0.15, -0.15]
        
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        result = self.gripper_client.get_result()
        if result.success: print "Gripper in position"
        else: print "Gripper positioning failed"
        
        goal = SmartArmGoal()
        goal.target_joints = [0.0, 1.77, 0.0, 0.0]
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result()
        result = self.arm_client.get_result()
        if result.success: print "Arm UP"
        else: print "Failed to raise arm"
        
        goal.target_joints = [0.93, -0.63, 0.0, 0.0]
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result()
        result = self.arm_client.get_result()
        if result.success: print "Arm ready to swat"
        else: print "Failed to position the arm for swatting"
        
        t = rospy.Time.now()
        if(self.fileOut != None):
		self.fileOut.write("Swatting action at " + str(t.secs) + "." + str(t.nsecs) + "\n")
        
        #self.sh_pan_speed_srv(3.0);
        self.inProgress = True
        goal.target_joints = [-0.93, -0.63, 0.0, 0.0]
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result()
        result = self.arm_client.get_result()
        if result.success: print "Arm just swatted"
        else: print "Arm failed to swat"
        
        #self.sh_pan_speed_srv(1.17);

if __name__ == '__main__':
	try:
		file = open('/home/robotlab/simData/simData.dat', 'w')
		swat = ObjectSwat(file)
		r = rospy.Rate(0.2)
        #while not rospy.is_shutdown():
		rospy.loginfo('There are %d objects in the world' % len(swat.all_objects))
           
		for idx in range(len(swat.all_objects)):
			for j in range(7):
				if swat.tuck_arm_for_navigation():
					if swat.go_to_object(idx):
						swat.position_self()
						swat.setBlockToSwat(swat.all_object_names[idx])
						swat.swat_object()
						swat.setBlockToSwat("")
		file.close()
		r.sleep()
	except rospy.ROSInterruptException:
		pass
