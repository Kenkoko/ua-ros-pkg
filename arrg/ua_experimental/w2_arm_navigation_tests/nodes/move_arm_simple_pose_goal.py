#!/usr/bin/env python

import roslib; roslib.load_manifest('w2_arm_navigation_tests')
import rospy

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_arm_msgs.msg import MoveArmAction
from move_arm_msgs.msg import MoveArmGoal
from motion_planning_msgs.msg import SimplePoseConstraint
from motion_planning_msgs.msg import PositionConstraint
from motion_planning_msgs.msg import OrientationConstraint
from geometric_shapes_msgs.msg import Shape


def pose_constraint_to_position_orientation_constraints(pose_constraint):
    position_constraint = PositionConstraint()
    orientation_constraint = OrientationConstraint()
    position_constraint.header = pose_constraint.header
    position_constraint.link_name = pose_constraint.link_name
    position_constraint.position = pose_constraint.pose.position

    position_constraint.constraint_region_shape.type = Shape.BOX
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

    position_constraint.constraint_region_orientation.x = 0.0
    position_constraint.constraint_region_orientation.y = 0.0
    position_constraint.constraint_region_orientation.z = 0.0
    position_constraint.constraint_region_orientation.w = 1.0

    position_constraint.weight = 1.0

    orientation_constraint.header = pose_constraint.header
    orientation_constraint.link_name = pose_constraint.link_name
    orientation_constraint.orientation = pose_constraint.pose.orientation
    orientation_constraint.type = pose_constraint.orientation_constraint_type

    orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
    orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
    orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
    orientation_constraint.weight = 1.0

    return position_constraint, orientation_constraint

def add_goal_constraint_to_move_arm_goal(pose_constraint, move_arm_goal):
    position_constraint, orientation_constraint = pose_constraint_to_position_orientation_constraints(pose_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

if __name__ == '__main__':
    rospy.init_node('move_arm_simple_pose_goal_node', anonymous=True)
    move_arm_client = SimpleActionClient('move_left_arm', MoveArmAction)
    
    rospy.loginfo('init_node complete')
    move_arm_client.wait_for_server()
    rospy.loginfo('Connected to move_left_arm action server')
    
    goal = MoveArmGoal()
    goal.planner_service_name = 'ompl_planning/plan_kinematic_path'
    goal.motion_plan_request.planner_id = ''
    goal.motion_plan_request.group_name = 'left_arm'
    goal.motion_plan_request.num_planning_attempts = 1
    goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
    
    desired_pose = SimplePoseConstraint()
    desired_pose.header.frame_id = "base_footprint";
    desired_pose.link_name = "L7_wrist_roll_link";
    desired_pose.pose.position.x = 0.571;
    desired_pose.pose.position.y = 0.070;
    desired_pose.pose.position.z = 0.105;

    desired_pose.pose.orientation.x = -0.46;
    desired_pose.pose.orientation.y = -0.30;
    desired_pose.pose.orientation.z = 0.68;
    desired_pose.pose.orientation.w = -0.46;

    desired_pose.absolute_position_tolerance.x = 0.04;
    desired_pose.absolute_position_tolerance.y = 0.04;
    desired_pose.absolute_position_tolerance.z = 0.04;

    desired_pose.absolute_roll_tolerance = 0.05;
    desired_pose.absolute_pitch_tolerance = 0.05;
    desired_pose.absolute_yaw_tolerance = 0.05;
    
    add_goal_constraint_to_move_arm_goal(desired_pose, goal)
    
    move_arm_client.send_goal(goal)
    finished_within_time = move_arm_client.wait_for_result(rospy.Duration(200.0))

    if not finished_within_time:
        move_arm_client.cancel_goal()
        rospy.loginfo("Timed out achieving goal A")
    else:
        state = move_arm_client.get_state()

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('Action finished and was successful.')
        else:
            rospy.loginfo('Action failed: %d'%(state))


#    init = (0.0,) * 7
#    tucked = (-1.9715, -1.7406, 0.0213, -0.1807, -1.8408, 1.0840, 0.1483)
#    sample1 = (0.0216, - 1.1143, 0.1235, 0.9809, 2.6989, -1.7704, 0.2139)
#    sample2 = (-0.7155, 0.0364, 1.0405, 0.1331, 0.4470, -0.3615, -0.4405)
#    
#    goal_poses = (init, sample1, sample2, tucked)
#    num_goals = len(goal_poses)
#    
#    for k in range(num_goals*1):
#        goal_pos = goal_poses[k%num_goals]
#        
#        for i, value in enumerate(goal_pos):
#            goal.motion_plan_request.goal_constraints.joint_constraints[i].position = value
#            
#        move_arm_client.send_goal(goal)
#        finished_within_time = move_arm_client.wait_for_result(rospy.Duration(200.0))
#        
#        if not finished_within_time:
#            move_arm_client.cancel_goal()
#            rospy.loginfo('Timed out trying to achieve a joint goal')
#        else:
#            state = move_arm_client.get_state()
#            if state == GoalStatus.SUCCEEDED:
#                rospy.loginfo('Action finished: %s' % str(state))
#            else:
#                rospy.loginfo('Action failed: %s' % str(state))

