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

import roslib
roslib.load_manifest('wubble_controllers')

import rospy
from core.serial_proxy import SerialProxy

class SerialBusDriverManager:
    def __init__(self):
        rospy.init_node('ttyUSB1_manager', anonymous=False)

        port_name = rospy.get_param('~port_name', '/dev/ttyUSB1')
        baud_rate = rospy.get_param('~baud_rate', 1000000)
        min_motor_id = rospy.get_param('~min_motor_id', 1)
        max_motor_id = rospy.get_param('~max_motor_id', 25)
        update_rate = rospy.get_param("~update_rate", 5)

        self.serial_proxy = SerialProxy(port_name, baud_rate, min_motor_id, max_motor_id, update_rate)
        self.serial_proxy.connect()
        
        rospy.on_shutdown(self.serial_proxy.disconnect)
        
        self.drivers = {}

    def start_driver(self, driver_name):
        if driver_name in self.drivers:
            rospy.loginfo('Driver already started. If you want to restart it, call restart.')
            return
        try:
            driver = __import__(driver_name)
        except ImportError, ie:
            rospy.logwarn('Unable to start driver ' + driver_name)
            rospy.logwarn(ie)
            return
        
        control = driver.DriverControl(out_cb=self.serial_proxy.queue_new_packet, in_cb=self.serial_proxy.get_motor_states)
        control.start()
        self.drivers[driver_name] = control
        rospy.loginfo('Driver %s successfully started.' %driver_name)

    def stop_driver(self, driver_name):
        if driver_name in self.drivers:
            self.drivers[driver_name].stop()
            del self.drivers[driver_name]
            rospy.loginfo('Driver %s successfully stopped.' %driver_name)
        else:
            rospy.loginfo('Driver was not running.')

    def restart_driver(self, driver_name):
        self.stop_driver(driver_name)
        self.start_driver(driver_name)

if __name__ == '__main__':
    try:
        manager = SerialBusDriverManager()
        manager.start_driver('laser_tilt_ax12')
        rospy.spin()
        manager.stop_driver('laser_tilt_ax12')
    except rospy.ROSInterruptException: pass
