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
Simplified Version of RFID class
"""
import roslib
roslib.load_manifest('phidgets')

import rospy
from phidgets.msg import RFIDEvent

try:
    #Phidget specific imports
    from Phidgets.PhidgetException import *
    from Phidgets.Events.Events import *
    from Phidgets.Devices.RFID import RFID
except ImportError, ie:
    print 'You must install the phidgets drivers and ensure the python bindings are in your PYTHONPATH'
    sys.exit(1)

class PhidgetsRFID (RFID):

    def __init__(self):
        """
        Subclass of RFID class provided by Phidgets Inc. It simplifies the calls to connnect
        and disconnect a phidget. It publishes rfid tag gained/lost events to the ROS topic "rfid".
        """
        rospy.init_node('rfid_publisher', anonymous=True)
        RFID.__init__(self)
        self.attachDevice()
        self.setAntennaOn(True)
        self.__publisher = rospy.Publisher('rfid', RFIDEvent)
        self.setGainedTagHandler(self.__tagGainedHandler)
        self.setLostTagHandler(self.__tagLostHandler)

    def __tagGainedHandler(self, tagEvent):
        """
        Simple handler for tag gained event. Turns on the onbaord LED.
        """
        self.setLEDOn(True)
        self.__publisher.publish(1, str(tagEvent.tag))

    def __tagLostHandler(self, tagEvent):
        """
        Simple handler for tag lost event. Turns off the onbaord LED.
        """
        self.setLEDOn(False)
        self.__publisher.publish(0, str(tagEvent.tag))

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
            print 'Please attach RFID reader now. (waiting 15 seconds)'
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

    def setGainedTagHandler(self, handler):
        """
        Helper method that checks the argument to ensure it is a method before
        setting the onTagHandler.
        """
        if callable(handler):
            try:
                self.setOnTagHandler(handler)
            except PhidgetException, e:
                handlePhidgetException(e)

    def setLostTagHandler(self, handler):
        """
        Helper method that checks the argument to ensure it is a method before
        setting the onTagLostHandler.
        """
        if callable(handler):
            try:
                self.setOnTagLostHandler(handler)
            except PhidgetException, e:
                handlePhidgetException(e)

    def getDeviceInfo(self):
        """
        Helper method that returns all the device info in a tuple.
        """
        return (self.isAttached(), self.getDeviceType(), self.getSerialNum(), \
        self.getDeviceVersion(), self.getOutputCount(), self.getAntennaOn(), self.getLEDOn())

    def printDeviceInfo(self):
        """
        Method to print the info returned from the getDeviceInfo method in a readable table form.
        """
        print """
|------------|----------------------------------|--------------|------------|
|- Attached -|-              Type              -|- Serial No. -|-  Version -|
|- %8s -|- %30s -|- %10d -|- %8d -|
|------------|----------------------------------|--------------|------------|
Number of outputs: %i -- Antenna Status: %s -- Onboard LED Status: %s
""" %self.getDeviceInfo()


# main -- allows to use with command rosrun rcs PhidgetsRFID.py
if __name__ == '__main__':
    PhidgetsRFID()
    print 'Use ctrl + c to stop...'
    rospy.spin()