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
NAME = 'actions_demo'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionClient
from wubble_actions.msg import *


def move_head(head_pan, head_tilt):
    goal = WubbleHeadGoal()
    goal.target_joints = JointsCommand()
    goal.target_joints.joints = [head_pan, head_tilt]

    client.send_goal(goal)
    client.wait_for_goal_to_finish()

    return client.get_result()


def look_at(frame_id, x, y, z):
    goal = WubbleHeadGoal()
    goal.target_point.x = x
    goal.target_point.y = y
    goal.target_point.z = z
    goal.target_point.frame_id = frame_id

    client.send_goal(goal)
    client.wait_for_goal_to_finish()

    return client.get_result()


def move_arm(shoulder_pan, shoulder_tilt, elbow_tilt, wrist_rotate):
    goal = SmartArmGoal()
    goal.target_joints = JointsCommand()
    goal.target_joints.joints = [shoulder_pan, shoulder_tilt, elbow_tilt, wrist_rotate]

    client.send_goal(goal)
    client.wait_for_goal_to_finish()

    return client.get_result()


def reach_at(frame_id, x, y, z):
    goal = SmartArmGoal()
    goal.target_point.x = x
    goal.target_point.y = y
    goal.target_point.z = z
    goal.target_point.frame_id = frame_id

    client.send_goal(goal)
    client.wait_for_goal_to_finish()

    return client.get_result()


def move_gripper(left_finger, right_finger):
    goal = SmartArmGripperGoal()
    goal.target_joints = JointsCommand()
    goal.target_joints.joints = [left_finger, right_finger]

    client.send_goal(goal)
    client.wait_for_goal_to_finish()

    return client.get_result()


def tilt_laser(n=1):
    goal = HokuyoLaserTiltGoal()
    goal.tilt_cycles = n

    client.send_goal(goal)
    client.wait_for_goal_to_finish()

    return client.get_result()



if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        client = SimpleActionClient("wubble_head_action", WubbleHeadAction)
        client.wait_for_server()

        print "Look forward and slightly up"
        result = look_at("/base_footprint", 5.0, 0.0, 3.0);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: [" + str(result.head_position[0]) + ", " + str(result.head_position[1]) + "]"

        print "Tilt 3 cycles"
        result = tilt(3);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: " + str(result.tilt_position)

        print "Reset arm to cobra"
        result = move_arm(0.0, 1.972222, -1.972222, 0.0);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: [" + str(result.arm_position[0]) + ", " + str(result.arm_position[1]) + \
                str(result.arm_position[2]) + ", " + str(result.arm_position[3]) + "]"

        print "Close gripper"
        result = move_gripper(-1.0, 1.0);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: [" + str(result.gripper_position[0]) + ", " + str(result.gripper_position[1]) + "]"


    except rospy.ROSInterruptException:
        pass

