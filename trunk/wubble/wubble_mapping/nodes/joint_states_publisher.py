#!/usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.  * Redistributions
#     in binary form must reproduce the above copyright notice, this list of
#     conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution. # Neither the name of
#     the Willow Garage, Inc. nor the names of its contributors may be used to
#     endorse or promote products derived from this software without specific
#     prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

PKG = 'wubble_mapping'
NAME = 'joint_states_publisher'

import roslib; roslib.load_manifest(PKG)
import rospy

from sensor_msgs.msg import JointState
from wubble_controllers.msg import JointStateList

import math


class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort


class JointStatesPublisher():
    def __init__(self):
        # Initialize node
        rospy.init_node(NAME, anonymous=True)

        # Initialize joint_states dictionary
        self.joint_states = dict()
        
        # Start controller state subscribers
        rospy.Subscriber('arm_controller/state', JointStateList, self.controller_state_handler)
        rospy.Subscriber('camera_pan_tilt_controller/state', JointStateList, self.controller_state_handler)
        rospy.Subscriber('laser_tilt_controller/state', JointStateList, self.controller_state_handler)
        
        # Start publisher
        self.joint_states_pub = rospy.Publisher('joint_states', JointState)


    def controller_state_handler(self, data):
        for joint in data:
            self.joint_states[joint.name] = JointStateObject(joint.name, joint.angle * math.pi / 180, 0.0, 0.0)

        # Construct message & publish joint states
        msg = JointState()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        msg.effort = list()
        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            #msg.velocity.append(joint.velocity)        # Field is optional --> can be left empty
            #msg.effort.append(joint.effort)            # Field is optional --> can be left empty
        msg.header.stamp = rospy.Time.now()
        self.joint_states_pub.publish(self.joint_states)


if __name__ == '__main__':
    try:
        s = JointStatesPublisher()            
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

