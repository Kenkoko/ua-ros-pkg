#!/usr/bin/env python

PKG = 'wubble_environments'

import roslib; roslib.load_manifest(PKG)

import rospy
from gazebo_plugins.msg import WorldState

def callback(data):
    rospy.loginfo(rospy.get_name()+' World State: ')
    for i in range(len(data.name)):
        pos = data.pose[i].position
        ori = data.pose[i].orientation
        lin = data.twist[i].linear
        ang = data.twist[i].angular
        frc = data.wrench[i].force
        trq = data.wrench[i].torque
        bbmin = data.boundsMin[i]
        bbmax = data.boundsMax[i]
        rospy.loginfo('\nName: %s',data.name[i])
        rospy.loginfo(' Pose: \n  Position: %s %s %s',pos.x,pos.y,pos.z)
        rospy.loginfo('  Orientation: %s %s %s %s',ori.x,ori.y,ori.z,ori.w)
        rospy.loginfo(' Twist: \n  Linear: %s %s %s',lin.x,lin.y,lin.z)
        rospy.loginfo('  Angular: %s %s %s',ang.x,ang.y,ang.z)
        rospy.loginfo(' Wrench: \n  Force: %s %s %s',frc.x,frc.y,frc.z)
        rospy.loginfo('  Torque: %s %s %s',trq.x,trq.y,trq.z)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('gazebo_world_state', WorldState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

