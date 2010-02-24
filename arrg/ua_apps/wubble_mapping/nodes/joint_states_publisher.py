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

import roslib; roslib.load_manifest('wubble_mapping')
import rospy

from sensor_msgs.msg import JointState
from ua_controller_msgs.msg import JointStateList

import math

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatesPublisher():
    def __init__(self):
        rospy.init_node('wubble_joint_states_publisher', anonymous=True)
        self.joint_states = {'base_caster_support_joint': JointStateMessage('base_caster_support_joint', 0.0, 0.0, 0.0),
                             'caster_wheel_joint': JointStateMessage('caster_wheel_joint', 0.0, 0.0, 0.0)}
        
        # Start controller state subscribers
        rospy.Subscriber('arm_controller/state', JointStateList, self.controller_state_handler)
        rospy.Subscriber('camera_pan_tilt_controller/state', JointStateList, self.controller_state_handler)
        rospy.Subscriber('laser_tilt_controller/state', JointStateList, self.controller_state_handler)
        
        # Start publisher
        self.joint_states_pub = rospy.Publisher('joint_states', JointState)
        
        publish_rate = rospy.get_param('~rate', 50)
        r = rospy.Rate(publish_rate)
        
        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()
            
    def controller_state_handler(self, data):
        for joint in data.motor_states:
            self.joint_states[joint.name] = JointStateMessage(joint.name,
                                                              math.radians(joint.angle),
                                                              math.radians(joint.speed),
                                                              0.0)
                                                              
    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointState()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        msg.effort = list()
        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(joint.velocity)        # Field is optional --> can be left empty
            #msg.effort.append(joint.effort)            # Field is optional --> can be left empty
        msg.header.stamp = rospy.Time.now()
        self.joint_states_pub.publish(msg)

if __name__ == '__main__':
    try:
        s = JointStatesPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

