#!/usr/bin/env python
import roslib; roslib.load_manifest('simulator_experiments')

import sys

import rospy
from gazebo_plugins.srv import *

def spawn_model_client(name, xml):
    rospy.wait_for_service('spawn_model')
    try:
        spawn_model = rospy.ServiceProxy('spawn_model', SpawnModel)
    
        sp = SpawnModelRequest()
        sp.model.model_name = name
        sp.model.robot_model = xml
        sp.model.xml_type = 1
        sp.model.robot_namespace = '/'
        sp.model.initial_pose.position.x = 0
        sp.model.initial_pose.position.y = 0
        sp.model.initial_pose.position.z = 0.2
        sp.model.initial_pose.orientation.w = 1

        resp1 = spawn_model(sp)
        print resp1.success
        print resp1.status_message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    xml_file = open(roslib.packages.find_resource("simulator_experiments", "blue_box.xml")[0])
#    xml_file = open(roslib.packages.find_resource("simulator_experiments", "object_model.urdf")[0])
    gazebo_xml = xml_file.read()
    spawn_model_client('test_object', gazebo_xml)
