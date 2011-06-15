#!/usr/bin/env python
# -*- coding: utf-8 -*-


import roslib
roslib.load_manifest('w2_arm_navigation_tests')

import rospy
import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction
from pr2_controllers_msgs.msg import JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class ActionTrajectoryTest:
    def __init__(self):
        self.traj_client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self.traj_client.wait_for_server()

    def start_trajectory(self, goal):
        # When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        self.traj_client.send_goal(goal)
        self.traj_client.wait_for_result()

    def test_trajectory(self):
        # our goal variable
        goal = JointTrajectoryGoal()

        # First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.append("upper_tilt_joint")
        goal.trajectory.joint_names.append("lower_tilt_joint")

        # We will have two waypoints in this goal trajectory

        # First trajectory point
        # To be reached 1 second after starting along the trajectory
        point = JointTrajectoryPoint()
        point.positions.append(1.9)
        point.positions.append(1.9)
        point.velocities.append(0.0)
        point.velocities.append(0.0)
        point.time_from_start = rospy.Duration(3.0)
        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions.append(-2.0)
        point.positions.append(-2.0)
        point.velocities.append(0.0)
        point.velocities.append(0.0)
        point.time_from_start = rospy.Duration(6.0)
        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions.append(1.0)
        point.positions.append(-1.0)
        point.velocities.append(0.0)
        point.velocities.append(0.0)
        point.time_from_start = rospy.Duration(8.0)
        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions.append(0.0)
        point.positions.append(0.0)
        point.velocities.append(0.0)
        point.velocities.append(0.0)
        point.time_from_start = rospy.Duration(9.0)
        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions.append(1.0)
        point.positions.append(-1.0)
        point.velocities.append(0.0)
        point.velocities.append(0.0)
        point.time_from_start = rospy.Duration(10.0)
        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions.append(0.0)
        point.positions.append(0.0)
        point.velocities.append(0.0)
        point.velocities.append(0.0)
        point.time_from_start = rospy.Duration(10.5)
        goal.trajectory.points.append(point)

        #we are done; return the goal
        return goal

    def get_state(self):
        return self.traj_client.get_state()

if __name__ == '__main__':
    rospy.init_node('trajectory_action_controller_node', anonymous=True)

    att = ActionTrajectoryTest()
    i = 100
    while i > 0:
        att.start_trajectory(att.test_trajectory())
        i -= 1
