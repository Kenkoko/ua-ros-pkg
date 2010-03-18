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
# Author: Antons Rebguns
# Author: Cody Jorgensen
# Author: Cara Slutter

import roslib
roslib.load_manifest('ax12_driver_core')

import os
import sys
import rospy

from ax12_driver_core.srv import *

def print_usage():
    print 'Usage: python controller_spawner.py (start|stop|restart) controller_name...'
    sys.exit(1)

def manage_controller(command, package_path, module_name, class_name, controller_name):
    if command.lower() == 'start':
        rospy.wait_for_service('start_controller')
        try:
            start_controller = rospy.ServiceProxy('start_controller', StartController)
            response = start_controller(package_path, module_name, class_name, controller_name)
            if response.success: rospy.loginfo(response.reason)
            else: rospy.logerr(response.reason)
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s' % e)
    elif command.lower() == 'stop':
        rospy.wait_for_service('stop_controller')
        try:
            stop_controller = rospy.ServiceProxy('stop_controller', StopController)
            response = stop_controller(controller_name)
            if response.success: rospy.loginfo(response.reason)
            else: rospy.logerr(response.reason)
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s' % e)
    elif command.lower() == 'restart':
        rospy.wait_for_service('restart_controller')
        try:
            restart_controller = rospy.ServiceProxy('restart_controller', RestartController)
            response = restart_controller(package_path, module_name, class_name, controller_name)
            if response.success: rospy.loginfo(response.reason)
            else: rospy.logerr(response.reason)
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s' % e)
    else:
        rospy.logerr('Invalid command.')
        print_usage()

if __name__ == '__main__':
    args = rospy.myargv()[1:]
    if len(args) < 2:
        print_usage()

    command = args[0]
    joint_controllers = args[1:]
    
    for controller_name in joint_controllers:
        try:
            controller = rospy.get_param(controller_name + '/controller')
        except KeyError, ke:
            rospy.logerr('Controller\'s [%s] configuration was not properly loaded on the parameter server' % controller_name)
            sys.exit(1)

        rospack = os.popen('rospack find ' + controller['package'])
        package_path = rospack.readline().strip()
        module_name = controller['module']
        class_name = controller['type']
        rospack.close()
        
        if not package_path:
            rospy.logerr('Controller\'s [%s] configuration specified package that does not exist' % controller_name)
            sys.exit(1)

        manage_controller(command, package_path, module_name, class_name, controller_name)

