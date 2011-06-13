#!/usr/bin/env python

import roslib; roslib.load_manifest('wubble2_robot')
import rospy

from actionlib import SimpleActionClient

from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.msg import GraspHandPostureExecutionGoal

if __name__ == '__main__':
    rospy.init_node('test_gripper_posture_controller', anonymous=True)
    posture_controller = SimpleActionClient('wubble_gripper_grasp_action', GraspHandPostureExecutionAction)
    posture_controller.wait_for_server()
    rospy.loginfo('got gripper_posture_controller')
    
    pg = GraspHandPostureExecutionGoal()
    pg.goal = GraspHandPostureExecutionGoal.RELEASE
    
    posture_controller.send_goal(pg)
    posture_controller.wait_for_result()
    
    rospy.loginfo('finished releasing')
    
    pg = GraspHandPostureExecutionGoal()
    pg.goal = GraspHandPostureExecutionGoal.GRASP
    
    posture_controller.send_goal(pg)
    posture_controller.wait_for_result()
    
    rospy.loginfo('finished grasping')
