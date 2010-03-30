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

from phidgets_ros.msg import AccelerometerEvent

#Phidget specific imports
try:
    from Phidgets.PhidgetException import *
    from Phidgets.Events.Events import *
    from Phidgets.Devices.Accelerometer import *
except ImportError, ie:
    rospy.logfatal("You must install the phidgets drivers and ensure the python bindings are in your PYTHONPATH")
    sys.exit(1)

class PhidgetsAccelerometer:
    def __init__(self):
        rospy.init_node('accelerometer', anonymous=True)
        self.name = rospy.get_param('~name', '')
        self.serial = rospy.get_param('~serial_number', -1)
        self.x_axis_id = rospy.get_param('~x_axis_id', 0)
        self.y_axis_id = rospy.get_param('~y_axis_id', 1)
        self.z_axis_id = rospy.get_param('~z_axis_id', 2)
        self.sensitivity = rospy.get_param('~sensitivity', 0.0)
        
        if self.serial == -1:
            rospy.logwarn("No serial number specified. This node will connect to the first accelerometer attached.")
        self.accelerometer = Accelerometer()

    def initialize(self):
        try:
            self.accelerometer.setOnAttachHandler(self.accelerometerAttached)
            self.accelerometer.setOnDetachHandler(self.accelerometerDetached)
            self.accelerometer.setOnErrorhandler(self.accelerometerError)
            self.accelerometer.setOnAccelerationChangeHandler(self.accelerationChanged)
            self.accelerometer.openPhidget(self.serial)
        except PhidgetException, e:
            rospy.logfatal("Phidget Exception %i: %s" % (e.code, e.message))
            sys.exit(1)

    def close(self):
        try:
            self.accelerometer.closePhidget()
        except PhidgetException, e:
            rospy.logfatal("Phidget Exception %i: %s" % (e.code, e.message))
            sys.exit(1)

    def accelerometerAttached(self, event):
        map(lambda x: self.accelerometer.setAccelChangeTrigger(x, self.sensitivity), range(self.accelerometer.getAxisCount()))
        if self.name:
            topic = 'accelerometer/%s' %self.name
        else:
            topic = 'accelerometer/%d' %event.device.getSerialNum()
        self.publisher = rospy.Publisher(topic, AccelerometerEvent)
        rospy.loginfo("Accelerometer %i Attached!" % (event.device.getSerialNum()))

    def  accelerometerDetached(self, event):
        self.publisher.unregister()
        rospy.loginfo("Accelerometer %i Detached!" % (event.device.getSerialNum()))

    def accelerometerError(self, error):
        rospy.logerr("Phidget Error %i: %s" % (error.eCode, error.description))

    def accelerationChanged(self, accelEvent):
        x_accel = self.accelerometer.getAcceleration(self.x_axis_id)
        y_accel = self.accelerometer.getAcceleration(self.y_axis_id)
        z_accel = 0
        if self.accelerometer.getAxisCount() > 2:
            z_accel = self.accelerometer.getAcceleration(self.z_axis_id)
        ae_msg = AccelerometerEvent()
        ae_msg.header.stamp = rospy.Time.now()
        ae_msg.acceleration_x = x_accel
        ae_msg.acceleration_y = y_accel
        ae_msg.acceleration_z = z_accel
        self.publisher.publish(ae_msg)

if __name__ == '__main__':
    try:
        pa = PhidgetsAccelerometer()
        pa.initialize()
        rospy.spin()
        pa.close()
    except rospy.ROSInterruptException: pass
