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

# Author: Anh Tran

PKG = 'wubble_actions'
NAME = 'move_base_demo'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from wubble_actions.msg import *


def move_to(frame_id, position, orientation, vicinity=0.0):
    goal = ErraticBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.pose.position = position
    goal.target_pose.pose.orientation = orientation
    goal.vicinity_range = vicinity

    client.send_goal(goal)
    client.wait_for_result()

    if client.get_state() == GoalStatus.SUCCEEDED:
        result = client.get_result()
        print "Result: " + str(result.base_position)
    elif client.get_state() == GoalStatus.PREEMPTED:
        print "Action pre-empted"
    else:
        print "Action failed"


if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        client = SimpleActionClient("erratic_base_action", ErraticBaseAction)
        client.wait_for_server()
        
        print "Go exactly 1 meter forward & 1 meter to the right..."
        position = Point(x=1.0, y=-1.0)
        orientation = Quaternion(w=1.0)
        move_to('/base_footprint', position, orientation) # No vicinity specified
        rospy.sleep(2.0)

        print "Go to rocky sphere..."
        position = Point(x=-2.6, y=1.7)
        orientation = Quaternion(w=1.0)
        move_to('/map', position, orientation, 0.9)
        rospy.sleep(2.0)

        print "Go to metal box..."
        position = Point(x=-2.8, y=-1.8)
        orientation = Quaternion()
        move_to('/map', position, orientation, 0.9)
        rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        pass

