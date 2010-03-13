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

import roslib
roslib.load_manifest('ax12_driver_core')

import sys
import rospy
from serial_proxy import SerialProxy
from ax12_driver_core.srv import *

class AX12DriverManager:
    def __init__(self):
        rospy.init_node('ax12_driver_manager', anonymous=False)

        port_name = rospy.get_param('~port_name', '/dev/ttyUSB0')
        baud_rate = rospy.get_param('~baud_rate', 1000000)
        min_motor_id = rospy.get_param('~min_motor_id', 1)
        max_motor_id = rospy.get_param('~max_motor_id', 25)
        update_rate = rospy.get_param("~update_rate", 5)

        self.serial_proxy = SerialProxy(port_name, baud_rate, min_motor_id, max_motor_id, update_rate)
        self.serial_proxy.connect()
        
        rospy.on_shutdown(self.serial_proxy.disconnect)
        
        self.drivers = {}
        
        rospy.Service('start_driver', DriverControl, self.start_driver)
        rospy.Service('stop_driver', DriverControl, self.stop_driver)
        rospy.Service('restart_driver', DriverControl, self.restart_driver)

    def start_driver(self, driver_control_request):
        driver_name = driver_control_request.driver_name
        driver_path = driver_control_request.driver_path
        joint_controllers = driver_control_request.joint_controllers
        # make sure the driver_path is in PYTHONPATH
        if not driver_path in sys.path:
            sys.path.append(driver_path)
        if driver_name in self.drivers:
            return DriverControlResponse(False, 'Driver already started. If you want to restart it, call restart.')
        try:
            if driver_name not in sys.modules:
                # import if module not previously imported
                driver = __import__(driver_name)
            else:
                # reload module if previously imported
                driver = reload(sys.modules[driver_name])
        except ImportError, ie:
            return DriverControlResponse(False, 'Cannot find driver module. Unable to start driver %s\n%s' %(driver_name, str(ie)))
        except SyntaxError, se:
            return DriverControlResponse(False, 'Syntax error in driver module. Unable to start driver %s\n%s' %(driver_name, str(se)))
        except Exception, e:
            return DriverControlResponse(False, 'Unknown error has occured. Unable to start driver %s\n%s' %(driver_name, str(e)))
           
        control = driver.DriverControl(self.serial_proxy.queue_new_packet, joint_controllers)
        if control.initialize():
            control.start()
            self.drivers[driver_name] = control
            return DriverControlResponse(True, 'Driver %s successfully started.' %driver_name)
        else:
            return DriverControlResponse(False, 'Initialization failed. Unable to start driver %s' %driver_name)

    def stop_driver(self, driver_control_request):
        driver_name = driver_control_request.driver_name
        if driver_name in self.drivers:
            self.drivers[driver_name].stop()
            del self.drivers[driver_name]
            return DriverControlResponse(True, 'Driver %s successfully stopped.' %driver_name)
        else:
            return DriverControlResponse(False, 'Driver was not running.')

    def restart_driver(self, driver_control_request):
        response1 = self.stop_driver(driver_control_request)
        response2 = self.start_driver(driver_control_request)
        return DriverControlResponse(response1.success and response2.success, '%s\n%s' %(response1.reason, response2.reason))

if __name__ == '__main__':
    try:
        manager = AX12DriverManager()
        rospy.spin()
    except rospy.ROSInterruptException: pass
