#!/usr/bin/env python

from random import random

import roslib
roslib.load_manifest('w2_object_manipulation_launch')

import rospy

from planar import Vec2, BoundingBox

from bolt_representation.speaker import Speaker
from bolt_representation.landmark import RectangleRepresentation, Scene, Landmark

from bolt_msgs.msg import Scene as BoltScene
from bolt_msgs.msg import MoveIt

from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32


def bolt_to_world(location, z=0.0):
    return Point32(location.y, -location.x, z)

def world_to_bolt(point):
    return Vec2(-point.y, point.x)


if __name__ == '__main__':
    rospy.init_node('bolt_to_world_command', anonymous=True)
    rospy.sleep(1)
    move_it_pub = rospy.Publisher('bolt_move_it', MoveIt)
    rospy.sleep(1)
    
    o_min = Vec2(0.04, 0.54)
    o_max = Vec2(0.088, 0.598)
    
    location = Vec2(0.16, 0.48)
    
    mit = MoveIt()
    mit.target_location = bolt_to_world(location, 0.25)
    mit.object_bbox = Polygon([bolt_to_world(o_min), bolt_to_world(o_max)])
    
    move_it_pub.publish(mit)

