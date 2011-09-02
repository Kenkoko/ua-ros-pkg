#!/usr/bin/env python

import roslib; roslib.load_manifest('w2_arm_navigation_tests')
import rospy

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import MoveArmGoal
from arm_navigation_msgs.msg import JointConstraint
from arm_navigation_msgs.msg import LinkPadding

if __name__ == '__main__':
    rospy.init_node('move_arm_simple_joint_goal_node', anonymous=True)
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
    
    links = ('L7_wrist_roll_link',
             'L6_wrist_pitch_link',
             'L5_forearm_roll_link',
             'L4_elbow_flex_link',
             'L3_upperarm_roll_link',
             'L2_shoulder_pan_link')
             
    joints = ('shoulder_pitch_joint',
              'shoulder_pan_joint',
              'upperarm_roll_joint',
              'elbow_flex_joint',
              'forearm_roll_joint',
              'wrist_pitch_joint',
              'wrist_roll_joint')
              
    goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint(j, 0.0, 0.1, 0.1, 0.0) for j in joints]
    #goal.motion_plan_request.link_padding = [LinkPadding(l,0.0) for l in links]
    
    init = (0.0,) * 7
    #pick = (0.08297109,-1.29328384, 0.01069185,-0.14805498, 0.14074285, 1.67430316, -1.56874745)
    tucked = (-1.9715, -1.7406, 0.0213, -0.1807, -1.8408, 1.0840, 0.1483)
    #tucked = (-1.837, -1.823, 0.175, -0.195, -1.861, 1.089, 0.128)
    sample1 = (0.0216, - 1.1143, 0.1235, 0.9809, 2.6989, -1.7704, 0.2139)
    sample2 = (-0.7155, 0.0364, 1.0405, 0.1331, 0.4470, -0.3615, -0.4405)
    pick = (0.10373755, -0.25866649,  0.02566044, -0.1677145,  -0.32607513,  1.3214246, 1.60355412)
    pick1 = (0.08126654, -0.6564465,   0.01710696, -1.04984376,  0.3096493,   1.87000244, -0.34305726)
    pick2 = (0.02634035, -0.60330119,  0.00962267, -0.4319474,  -0.06141874,  1.67671771, 0.46094389)
    pick3 = (0.33116126, -1.68955237,  0.01603777,  0.53076162,  1.33585764,  1.26646647, -1.52117606)
    pick4 = (0.43867932, -1.6539393,   0.03100637,  0.6332176,   1.71315069,  0.9762375, 1.47422282)
    place = (0.261, -0.704, 1.470, 0.337, 0.910, -1.667, -0.026)
    new_init = (-1.650, -1.465, 3.430, -0.970, -1.427,  0.337,  0.046)
    
    goal_poses = (tucked,)# sample1, sample2, tucked)
    num_goals = len(goal_poses)
    
    for k in range(num_goals*1):
        goal_pos = goal_poses[k%num_goals]
        
        for i, value in enumerate(goal_pos):
            goal.motion_plan_request.goal_constraints.joint_constraints[i].position = value
            
        move_arm_client.send_goal(goal)
        finished_within_time = move_arm_client.wait_for_result(rospy.Duration(200.0))
        
        if not finished_within_time:
            move_arm_client.cancel_goal()
            rospy.loginfo('Timed out trying to achieve a joint goal')
        else:
            state = move_arm_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo('Action finished: %s' % str(state))
            else:
                rospy.loginfo('Action failed: %s' % str(state))
                
        rospy.sleep(2)

