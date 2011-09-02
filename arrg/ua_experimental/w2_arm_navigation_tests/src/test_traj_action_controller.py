#!/usr/bin/env python
# -*- coding: utf-8 -*-


import roslib
roslib.load_manifest('w2_arm_navigation_tests')

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class ActionTrajectoryTest:
    def __init__(self):
        self.traj_client = actionlib.SimpleActionClient('l_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.traj_client.wait_for_server()

    def start_trajectory(self, goal):
        # When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        self.traj_client.send_goal(goal)
        self.traj_client.wait_for_result()

    def test_trajectory(self):
        # our goal variable
        goal = FollowJointTrajectoryGoal()

        # First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names = ('shoulder_pitch_joint',
                                       'shoulder_pan_joint',
                                       'upperarm_roll_joint',
                                       'elbow_flex_joint',
                                       'forearm_roll_joint',
                                       'wrist_pitch_joint',
                                       'wrist_roll_joint')

        # We will have two waypoints in this goal trajectory

        # First trajectory point
        # To be reached 1 second after starting along the trajectory
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 7
        point.velocities = [1.0] * 7
        point.time_from_start = rospy.Duration(3.0)
        goal.trajectory.points.append(point)

        #we are done; return the goal
        return goal

    def get_state(self):
        return self.traj_client.get_state()

if __name__ == '__main__':
    rospy.init_node('trajectory_action_controller_node', anonymous=True)

    att = ActionTrajectoryTest()
    i = 1
    while i > 0:
        att.start_trajectory(att.test_trajectory())
        i -= 1
        
