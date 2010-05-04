#!/usr/bin/env python
import roslib; roslib.load_manifest('simulator_experiments')

import sys
import time

import rospy
from gazebo_plugins.srv import *

def spawn_model_client(name, xml):
    rospy.wait_for_service('add_model')
    try:
        spawn_model = rospy.ServiceProxy('add_model', SpawnModel)
    
        sp = SpawnModelRequest()

#        sp.model.model_name = name
        sp.model.robot_model = xml
        
        sp.model.xml_type = sp.model.GAZEBO_XML

        sp.model.robot_namespace = '/'
        
        sp.model.initial_pose.position.x = 0
        sp.model.initial_pose.position.y = 0
        sp.model.initial_pose.position.z = 0.3
        sp.model.initial_pose.orientation.w = 1

        resp1 = spawn_model(sp)
        print resp1.success
        print resp1.status_message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

count = 0
def spawn_model(file_name):
    global count
    xml_file = open(roslib.packages.find_resource("simulator_experiments", file_name)[0])
    gazebo_xml = xml_file.read()
    count += 1
    spawn_model_client('test_object' + str(count), gazebo_xml)

if __name__ == "__main__":
    spawn_model("blue_sphere.xml")
    time.sleep(3)
    spawn_model("blue_box.xml")
