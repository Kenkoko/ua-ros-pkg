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

import os
import sys
import yaml
import rospy

from srv import *

def load_params(file_path):
    if os.path.exists(file_path):
        rospy.set_param('', yaml.load(open(file_path)))
    else:
        print 'No param file found at:', file_path

def print_usage():
    print 'Usage: python driver_ctrl.py [start|stop|restart] absolute_path_to_driver <yaml_file_with_params>'
    sys.exit(1)

if __name__ == '__main__':
    args = sys.argv[1:]
    if len(args) < 2:
        print_usage()
    command = args[0]
    abs_driver_path = args[1]
    if not os.path.exists(abs_driver_path):
        print 'No driver found at path: %s' %abs_driver_path
        print_usage()
    driver_path, driver_name = os.path.split(abs_driver_path)
    driver_name = os.path.splitext(driver_name)[0]
    if command.lower() == 'start':
        if len(args) == 3:
            load_params(args[2])
        rospy.wait_for_service('start_driver')
        try:
            start_driver = rospy.ServiceProxy('start_driver', DriverControl)
            response = start_driver(driver_name, driver_path)
            if response.success:
                rospy.loginfo(response.reason)
            else:
                rospy.logerr(response.reason)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    elif command.lower() == 'stop':
        rospy.wait_for_service('stop_driver')
        try:
            stop_driver = rospy.ServiceProxy('stop_driver', DriverControl)
            response = stop_driver(driver_name, driver_path)
            if response.success:
                rospy.loginfo(response.reason)
            else:
                rospy.logerr(response.reason)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    elif command.lower() == 'restart':
        if len(args) == 3:
            load_params(args[2])
        rospy.wait_for_service('restart_driver')
        try:
            restart_driver = rospy.ServiceProxy('restart_driver', DriverControl)
            response = restart_driver(driver_name, driver_path)
            if response.success:
                rospy.loginfo(response.reason)
            else:
                rospy.logerr(response.reason)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    else:
        print 'Invalid command.'
        print_usage()
    
    
        
