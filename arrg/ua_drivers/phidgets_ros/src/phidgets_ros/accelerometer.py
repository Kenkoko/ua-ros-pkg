#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Arizona Robotics Research Group,
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

"""
Simplified Version of Accelerometer class
"""
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
    print 'You must install the phidgets drivers and ensure the python bindings are in your PYTHONPATH'
    sys.exit(1)

class PhidgetsAccelerometer (Accelerometer):
    """
    Subclass of Accelerometer class provided by Phidgets Inc. It simplifies the calls to connnect
    and disconnect a phidget. It publishes acelerometer value changed events to the ROS topic "accelerometer".
    """

    def __init__(self, sensitivity=0.500):
        """
        Calls the initialize method and the attach method. Allows the client to specify the
        sensitivity for the accelerometer value changed events.
        """
        rospy.init_node('accelerometer_publisher', anonymous=True)
        Accelerometer.__init__(self)
        self.attachDevice()
        self.setOnAccelerationChangeHandler(self.__accelChanged)
        try:
            numAxis = self.getAxisCount()
            self.setAccelChangeTrigger(0, sensitivity)
            self.setAccelChangeTrigger(1, sensitivity)
            if numAxis > 2:
                self.setAccelChangeTrigger(2, sensitivity)
        except PhidgetException, e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print "Exiting..."
            exit(1)
        self.__publisher = rospy.Publisher('accelerometer', AccelerometerEvent)

    def __accelChanged(self, accelEvent):
        """
        Simpler handler for an accelerometer value changed event.
        """
        self.__publisher.publish(accelEvent.index, accelEvent.acceleration)

    def attachDevice(self):
        """
        Handles calling openPhidget() with all the correct exception catching. Upon getting an exception
        the program will terminate.
        """
        try:
            self.openPhidget()
        except PhidgetException, e:
            print "Phidget Exception %i: %s" % (e.code, e.message)
            print "Exiting..."
            exit(1)
        try:
            print 'Please attach Accelerometer now. (waiting 15 seconds)'
            self.waitForAttach(15000)
            print 'Attached.'
        except PhidgetException, e:
            print "Phidget Exception %i: %s" % (e.code, e.message)
            try:
                self.closePhidget()
            except PhidgetException, e:
                print "Phidget Exception %i: %s" % (e.code, e.message)
                print "Exiting..."
                exit(1)
            print "Exiting..."
            exit(1)

    def closeDevice(self):
        """
        Handles calling closePhidget() with all the correct exception catching. Upon getting an exception
        the program will terminate.
        """
        try:
            self.closePhidget()
        except PhidgetException, e:
            print "Phidget Exception %i: %s" % (e.code, e.message)
            print "Exiting..."
            exit(1)

    def getDeviceInfo(self):
        """
        Helper method that returns all the device info in a tuple.
        """
        return (self.isAttached(), self.getDeviceType(), self.getSerialNum(), \
        self.getDeviceVersion(), self.getAxisCount())

    def printDeviceInfo(self):
        """
        Method to print the info returned from the getDeviceInfo method in a readable table form.
        """
        print """
|------------|----------------------------------|--------------|------------|
|- Attached -|-              Type              -|- Serial No. -|-  Version -|
|- %8s -|- %30s -|- %10d -|- %8d -|
|------------|----------------------------------|--------------|------------|
Number of Axes: %i
""" %self.getDeviceInfo()

# main -- allows use of command 'rosrun phidgets_ros accelerometer.py'
if __name__ == '__main__':
    PhidgetsAccelerometer()
    rospy.spin()
