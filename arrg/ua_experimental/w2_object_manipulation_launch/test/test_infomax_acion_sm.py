#!/usr/bin/env python

import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from w2_object_manipulation_launch.msg import DoInfomaxAction
from w2_object_manipulation_launch.msg import DoInfomaxGoal

if __name__ == '__main__':
    rospy.init_node('infomax_action_sm_tester')
    
    client = SimpleActionClient('do_infomax', DoInfomaxAction)
    client.wait_for_server()
    
    goal = DoInfomaxGoal()
    goal.graspable_object_name = 'graspable_object_1'
    
    result = client.send_goal_and_wait(goal)
    
    if result == GoalStatus.SUCCEEDED:
        rospy.loginfo('infomax action state machine successfully completed execution')
    else:
        rospy.loginfo('failed to infomax an object')

