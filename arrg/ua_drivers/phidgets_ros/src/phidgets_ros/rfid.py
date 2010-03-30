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

from phidgets_ros.msg import RFIDEvent

try:
    #Phidget specific imports
    from Phidgets.PhidgetException import *
    from Phidgets.Events.Events import *
    from Phidgets.Devices.RFID import *
except ImportError, ie:
    rospy.logfatal("You must install the phidgets drivers and ensure the python bindings are in your PYTHONPATH")
    sys.exit(1)

class PhidgetsRFID:
    def __init__(self):
        rospy.init_node('rfid', anonymous=True)
        self.name = rospy.get_param('~name', '')
        self.serial = rospy.get_param('~serial_number', -1)
        
        if self.serial == -1:
            rospy.logwarn("No serial number specified. This node will connect to the first rfid attached.")
        self.rfid = RFID()

    def initialize(self):
        try:
            self.rfid.setOnAttachHandler(self.rfidAttached)
            self.rfid.setOnDetachHandler(self.rfidDetached)
            self.rfid.setOnErrorhandler(self.rfidError)
            self.rfid.setOnTagHandler(self.rfidTagGained)
            self.rfid.setOnTagLostHandler(self.rfidTagLost)
            self.rfid.openPhidget(self.serial)
        except PhidgetException, e:
            rospy.logfatal("Phidget Exception %i: %s" % (e.code, e.message))
            sys.exit(1)

    def close(self):
        try:
            self.rfid.closePhidget()
        except PhidgetException, e:
            rospy.logfatal("Phidget Exception %i: %s" % (e.code, e.message))
            sys.exit(1)

    def rfidAttached(self, event):
        self.rfid.setAntennaOn(True)
        if self.name:
            topic = 'rfid/%s' %self.name
        else:
            topic = 'rfid/%d' %event.device.getSerialNum()
        self.publisher = rospy.Publisher(topic, RFIDEvent)
        rospy.loginfo("RFID %i Attached!" % (event.device.getSerialNum()))

    def rfidDetached(self, event):
        self.publisher.unregister()
        rospy.loginfo("RFID %i Detached!" % (event.device.getSerialNum()))

    def rfidError(self, error):
        rospy.logerr("Phidget Error %i: %s" % (error.eCode, error.description))

    def rfidTagGained(self, tagEvent):
        """
        Publishes a tag message and turns on the onboard LED.
        """
        self.rfid.setLEDOn(True)
        re_msg = RFIDEvent()
        re_msg.header.stamp = rospy.Time.now()
        re_msg.gained = True
        re_msg.tag = str(tagEvent.tag)
        self.publisher.publish(re_msg)

    def rfidTagLost(self, tagEvent):
        """
        Publishes a tag message and turns off the onboard LED.
        """
        self.rfid.setLEDOn(False)
        re_msg = RFIDEvent()
        re_msg.header.stamp = rospy.Time.now()
        re_msg.gained = False
        re_msg.tag = str(tagEvent.tag)
        self.publisher.publish(re_msg)

if __name__ == '__main__':
    try:
        pr = PhidgetsRFID()
        pr.initialize()
        rospy.spin()
        pr.close()
    except rospy.ROSInterruptException: pass
