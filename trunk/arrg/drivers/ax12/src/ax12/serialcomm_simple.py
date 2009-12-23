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

import rio
import time
import rospy

from ax12.msg import Move
from ax12.msg import MotorData

rospy.init_node('serialcomm_simple', anonymous=True)
while not rospy.has_param('robot/ax12/motors'):
    print 'System is not initialized, will try again in 3 seconds'
    time.sleep(3)

# get port and baudrate from param server
port = rospy.get_param('robot/ax12/port', '/dev/ttyUSB0')
baud = rospy.get_param('robot/ax12/baudrate', 1000000)

axio = rio.AX12_IO(port, baud)

# Get the available motor ids from the ROS parameter server.
# The "main" waits for the init_sys.py to place this information
# on the parameter server before starting this ROS node
motorIds = rospy.get_param('robot/ax12/motors')

# Initialize the publishers for the available motors
# Topics are named "robot/ax12/dynamixel" + motor_id
# Topic publishes messages of type bioloid/MotorData
publishers = {}
for id in motorIds:
    publishers[id] = rospy.Publisher('robot/ax12/dynamixel' + str(id), MotorData)

def handle_incoming_commands(command):
    axio.set_servo_position_and_speed(command.id, command.position, command.speed)

if __name__ == '__main__':
    # Add our handle_incoming_commands method as a subscriber to "robot/ax12/commands"
    rospy.Subscriber('robot/ax12/commands', Move, handle_incoming_commands)
    rospy.spin()
    