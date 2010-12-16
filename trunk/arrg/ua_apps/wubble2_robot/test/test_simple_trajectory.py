#!/usr/bin/env python
# -*- coding: utf-8 -*-


import roslib
roslib.load_manifest('wubble2_robot')

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
        goal.trajectory.joint_names.append('shoulder_pitch_joint')
        goal.trajectory.joint_names.append('shoulder_yaw_joint')
        goal.trajectory.joint_names.append('shoulder_roll_joint')
        goal.trajectory.joint_names.append('elbow_pitch_joint')
        goal.trajectory.joint_names.append('wrist_roll_joint')
        goal.trajectory.joint_names.append('wrist_pitch_joint')
        goal.trajectory.joint_names.append('wrist_yaw_joint')

        # We will have two waypoints in this goal trajectory

        # First trajectory point
        # To be reached 1 second after starting along the trajectory
#        point = JointTrajectoryPoint()
#        point.positions  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#        point.time_from_start = rospy.Duration(3.0)
#        goal.trajectory.points.append(point)

#        point = JointTrajectoryPoint()
#        point.positions =  [-1.90742604, -1.7149727400000001, 0.11761035, -0.32930898000000003, -1.830550302, 1.130032449, -0.020453076000000001]
#        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#        point.time_from_start = rospy.Duration(6.0)
#        goal.trajectory.points.append(point)

#        point = JointTrajectoryPoint()
#        point.positions  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#        point.time_from_start = rospy.Duration(9.0)
#        goal.trajectory.points.append(point)

#        point = JointTrajectoryPoint()
#        point.positions =  [-1.90742604, -1.7149727400000001, 0.11761035, -0.32930898000000003, -1.830550302, 1.130032449, -0.020453076000000001]
#        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#        point.time_from_start = rospy.Duration(12.0)
#        goal.trajectory.points.append(point)

        point = JointTrajectoryPoint()
        point.positions  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = rospy.Duration(3.0)
        goal.trajectory.points.append(point)

#        point = JointTrajectoryPoint()
#        point.positions  = [-0.37, -0.799431, -0.180564, -0.053818, -0.312018, 0.41247, 0.943618]
#        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#        point.time_from_start = rospy.Duration(14.0)
#        goal.trajectory.points.append(point)

        # pick up location 1
#        point = JointTrajectoryPoint()
#        point.positions  = [-2.0998793400000002, 1.0841535900000001, 1.98654573, 0.60836626500000002, 1.0533334140000001, -0.20453076000000001, 0.010226538]
#        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#        point.time_from_start = rospy.Duration(16.0)
#        goal.trajectory.points.append(point)

        # pick up location 2
        point = JointTrajectoryPoint()
        point.positions  = [0.45, -0.98, -0.11, 0.61, 0.21, -0.47, 0.18]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = rospy.Duration(5.0)
        goal.trajectory.points.append(point)

        # pick up location 2
        point = JointTrajectoryPoint()
        point.positions  = [0.45, -1.074, 0.087, -0.045, 0.509, -0.377, 1.029]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = rospy.Duration(7.0)
        goal.trajectory.points.append(point)

        # shake location
        point = JointTrajectoryPoint()
        point.positions  = [-1.1, -1.58025543, 0.52817739000000008, -0.75698298000000008, -1.130032449, -0.30679613999999999, -0.18407768400000002]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = rospy.Duration(18.0)
        goal.trajectory.points.append(point)

        #we are done; return the goal
        return goal

    def get_state(self):
        return self.traj_client.get_state()

if __name__ == '__main__':
    rospy.init_node('trajectory_action_controller_node', anonymous=True)

    att = ActionTrajectoryTest()
    att.start_trajectory(att.test_trajectory())

