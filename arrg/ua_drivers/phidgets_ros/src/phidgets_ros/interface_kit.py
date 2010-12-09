#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Arizona Robotics Research Group,
# University of Arizona. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Cody Jorgensen
# Author: Antons Rebguns
#

import roslib
roslib.load_manifest('phidgets_ros')

import sys
import rospy

from phidgets_ros.msg import Float64Stamped
from phidgets_ros.srv import SetDigitalOutState
from phidgets_ros.srv import GetDigitalOutState

#Phidget specific imports
try:
    from Phidgets.PhidgetException import *
    from Phidgets.Events.Events import *
    from Phidgets.Devices.InterfaceKit import *
except ImportError, ie:
    rospy.logfatal("You must install the phidgets drivers and ensure the python bindings are in your PYTHONPATH")
    sys.exit(1)

class PhidgetsInterfaceKit:
    def __init__(self):
        rospy.init_node('interface_kit', anonymous=True)
        self.name = rospy.get_param('~name', '')
        self.serial = rospy.get_param('~serial_number', -1)
        
        if self.serial == -1:
            rospy.logwarn("No serial number specified. This node will connect to the first interface kit attached.")
        self.interface_kit = InterfaceKit()

    def initialize(self):
        try:
            self.interface_kit.setOnAttachHandler(self.interfaceKitAttached)
            self.interface_kit.setOnDetachHandler(self.interfaceKitDetached)
            self.interface_kit.setOnErrorhandler(self.interfaceKitError)
            self.interface_kit.setOnSensorChangeHandler(self.interfaceKitSensorChanged)
            self.interface_kit.openPhidget(self.serial)
        except PhidgetException, e:
            rospy.logfatal("Phidget Exception %i: %s" % (e.code, e.message))
            sys.exit(1)

    def close(self):
        try:
            self.interface_kit.closePhidget()
        except PhidgetException, e:
            rospy.logfatal("Phidget Exception %i: %s" % (e.code, e.message))
            sys.exit(1)

    def interfaceKitAttached(self, event):
        sensors = range(self.interface_kit.getSensorCount())
        outputs = range(self.interface_kit.getOutputCount())
        map(lambda x: self.interface_kit.setSensorChangeTrigger(x, 0), sensors)
        if self.name: topic = 'interface_kit/%s' %self.name
        else: topic = 'interface_kit/%d' %event.device.getSerialNum()
        self.publishers = [rospy.Publisher('%s/sensor/%d' %(topic, x), Float64Stamped) for x in sensors]
        rospy.Service('%s/set_digital_out_state' % (topic), SetDigitalOutState, self.process_set_digital_out_state)
        rospy.Service('%s/get_digital_out_state' % (topic), GetDigitalOutState, self.process_get_digital_out_state)
        rospy.loginfo("InterfaceKit %i Attached!" % (event.device.getSerialNum()))

    def interfaceKitDetached(self, event):
        map(lambda pub: pub.unregister(), self.publishers)
        del self.publishers[:]
        rospy.loginfo("InterfaceKit %i Detached!" % (event.device.getSerialNum()))

    def interfaceKitError(self, error):
        rospy.logerr("Phidget Error %i: %s" % (error.eCode, error.description))

    def interfaceKitSensorChanged(self, event):
        fs_msg = Float64Stamped()
        fs_msg.header.stamp = rospy.Time.now()
        fs_msg.data = event.value
        self.publishers[event.index].publish(fs_msg)

    def process_set_digital_out_state(self, req):
        try:
            self.interface_kit.setOutputState(req.index, req.state)
        except PhidgetException as pe:
            rospy.logerr('Unable to set digital output state')
            return None
        return []

    def process_get_digital_out_state(self, req):
        try:
            state = self.interface_kit.getOutputState(req.index)
        except PhidgetException as pe:
            rospy.logerr('Unable to get digital output state')
            return None
        return state

if __name__ == '__main__':
    try:
        pik = PhidgetsInterfaceKit()
        pik.initialize()
        rospy.spin()
        pik.close()
    except rospy.ROSInterruptException: pass

