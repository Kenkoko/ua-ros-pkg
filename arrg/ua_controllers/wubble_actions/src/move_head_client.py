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
NAME = 'move_head_client'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionClient
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from wubble_actions.msg import *


def look_at(frame_id, x, y, z):

    # Creates a goal to send to the action server.
    goal = WubblePointHeadGoal()
    goal.point.x = x
    goal.point.y = y
    goal.point.z = z
    goal.frame_id = frame_id

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_goal_to_finish()

    # Return result
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        client = SimpleActionClient("wubble_point_head_action", WubblePointHeadAction)
        client.wait_for_server()

        print "Look forward and slightly up"
        result = look_at("/base_footprint", -5.0, 0.0, 3.0);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: [" + str(result.head_position[0]) + ", " + str(result.head_position[1]) + "]"

        print "Look to the right and slightly down"
        result = look_at("/base_footprint", 1.0, -5.0, 0.0);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: [" + str(result.head_position[0]) + ", " + str(result.head_position[1]) + "]"

        print "Look to the left and slightly up"
        result = look_at("/base_footprint", 1.0, 5.0, 2.0);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: [" + str(result.head_position[0]) + ", " + str(result.head_position[1]) + "]"

        print "Look at the gripper"
        result = look_at("/arm_left_finger_link", 0.0, 0.0, 0.0);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: [" + str(result.head_position[0]) + ", " + str(result.head_position[1]) + "]"

    except rospy.ROSInterruptException:
        pass

