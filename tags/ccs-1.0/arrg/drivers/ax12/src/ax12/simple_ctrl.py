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
roslib.load_manifest('ax12')

import sys
import rospy
from ax12.msg import Move

if __name__ == '__main__':
    
    rospy.init_node('simple_ctrl', anonymous=True)
    
    # list of words that will break loop
    exit = ['done', 'exit', 'no', 'quit', 'stop']
    print 'Type one of the following to exit: ' + str(exit).strip('[]')
    
    publisher = rospy.Publisher('robot/ax12/moves', Move)
    
    motorIds = rospy.get_param('robot/ax12/motors')
    
    while True:
    
        valid_motor_id = False
        while not valid_motor_id:
            print 'Choose one of the following motor ids: ' + str(motorIds)
            motor_id = raw_input('Enter motor id: ')
            if motor_id in exit:
                sys.exit(0)
            try:
                motor_id = int(motor_id)
            except ValueError:
                valid_motor_id = False
            else:
                if motor_id in motorIds:
                    valid_motor_id = True

        
        # Check if motor is in freespin
        min = rospy.get_param('robot/ax12/motor/%d/minAngle' %motor_id, 0)
        max = rospy.get_param('robot/ax12/motor/%d/maxAngle' %motor_id, 0)
        
        valid_motor_pos = False
        while not valid_motor_pos:
            if min == 0 and max == 0:
                position = 0
                valid_motor_pos = True
            else:
                position = raw_input('Enter a motor position: ')
                if position in exit:
                    sys.exit(0)
                try:
                    position = int(position)
                except ValueError:
                    valid_motor_pos = False
                else:
                    valid_motor_pos = True
        
        valid_motor_speed = False
        while not valid_motor_speed:
            speed = raw_input('Enter a motor speed: ')
            if speed in exit:
                sys.exit(0)
            try:
                speed = int(speed)
            except ValueError:
                valid_motor_speed = False
            else:
                valid_motor_speed = True

        m = Move(motor_id, position, speed)
        publisher.publish(m)