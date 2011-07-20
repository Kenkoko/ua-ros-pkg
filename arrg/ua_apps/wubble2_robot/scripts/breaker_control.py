#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
#

import sys
import time
from subprocess import call

import roslib; roslib.load_manifest('wubble2_robot')
import rospy

from phidgets_ros.srv import SetDigitalOutState

SERVICE_NAME = '/interface_kit/124427/set_digital_out_state'
MAP = {'ON': True, 'OFF': False}

def print_usage_and_exit():
    print 'Specify breaker state [on|off]'
    print 'Example: %s on' % rospy.myargv()[0]
    print 'Example: %s off' % rospy.myargv()[0]
    exit(1)

if __name__ == '__main__':
    if len(rospy.myargv()) == 2:
        breaker_state = rospy.myargv()[1].upper()
        
        if breaker_state not in MAP:
            print_usage_and_exit()
    else:
        print_usage_and_exit()
        
    rospy.init_node('breaker_control')
    
    rospy.loginfo('Waiting for %s...' % SERVICE_NAME)
    rospy.wait_for_service(SERVICE_NAME)
    breaker_control_srv = rospy.ServiceProxy(SERVICE_NAME, SetDigitalOutState)
    rospy.loginfo('Connected to %s' % SERVICE_NAME)
    
    # control 12V breaker (InterfaceKits, swissranger, etc.)
    rospy.loginfo('Switching 12V breaker %s' % breaker_state)
    breaker_control_srv(0, MAP[breaker_state])
    
    # control 16V breaker (wubble arm motors)
    rospy.loginfo('Switching 16V breaker %s' % breaker_state)
    breaker_control_srv(1, MAP[breaker_state])
    
    time.sleep(2.0)
    
    # switch to external mic if turning breakers on
    if MAP[breaker_state]:
        rospy.loginfo('Switching to external microphone')
        call(['pacmd', 'set-default-source', 'alsa_input.usb-Burr-Brown_from_TI_USB_Audio_CODEC-00-default.analog-stereo'])

