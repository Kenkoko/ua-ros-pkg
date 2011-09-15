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


PKG = 'wubble2_robot'
NAME = 'command_gripper'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionClient
from wubble2_robot.msg import *

def close_gripper_adaptive():
    # Creates a goal to send to the action server.
    goal = WubbleGripperGoal()
    goal.command = WubbleGripperGoal.CLOSE_GRIPPER
    goal.torque_limit = 0.4
    goal.dynamic_torque_control = True
    goal.pressure_upper = 1900.0
    goal.pressure_lower = 1800.0
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    client.wait_for_result()

def open_gripper():
    # Creates a goal to send to the action server.
    goal = WubbleGripperGoal()
    goal.command = WubbleGripperGoal.OPEN_GRIPPER
    goal.torque_limit = 0.6
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    client.wait_for_result()

def close_gripper():
    # Creates a goal to send to the action server.
    goal = WubbleGripperGoal()
    goal.command = WubbleGripperGoal.CLOSE_GRIPPER
    goal.torque_limit = 0.3
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    client.wait_for_result()

def close_gripper_gentle():
    close_gripper()
    
    # Creates a goal to send to the action server.
    goal = WubbleGripperGoal()
    goal.command = WubbleGripperGoal.CLOSE_GRIPPER
    goal.torque_limit = 0
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    client.wait_for_result()


if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        client = SimpleActionClient('wubble_gripper_action', WubbleGripperAction)
        client.wait_for_server()
        
        print "Open gripper"
        open_gripper()
        rospy.sleep(1)
        print "Close gripper"
        close_gripper()
        rospy.sleep(1)
        print "Open gripper"
        open_gripper()
        rospy.sleep(1)
        print "Adpative close gripper"
        close_gripper_adaptive()
        rospy.sleep(1)
        print "Gentle close gripper"
        close_gripper_gentle()
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass

