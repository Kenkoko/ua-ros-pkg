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
Simple script that pings AX12 motors to determine which motor IDs are connected.
After constructing a list of connected motor IDs, it retrieves each motor's min
and max angles of movement (the values in the motors registers for CW and CCW
angle limits). The list of motor IDs and each motor's min and max angles are
placed on the ROS parameter server.
"""

import roslib
roslib.load_manifest('ax12')

import sys
import rio
import rospy

# Change this to a larger number if your system has dynamixels with IDs larger
# than default value of 25
MAX_IDS_TO_PING = 25

USAGE = """
    Usage: %s [ path/to/serialport [baudrate] ]
    Examples:  %s /dev/ttyUSB0 1000000
               %s /dev/ttyUSB0
    Using ROS: rosrun package_name %s /dev/ttyUSB0 1000000
               rosrun package_name %s /dev/ttyUSB0
""" %( (sys.argv[0], ) * 5 )

if __name__ == '__main__':
    
    if len(sys.argv) == 2 and sys.argv[1] == '--help':
        print USAGE
        sys.exit(0)
    
    rospy.init_node('init_sys', anonymous=True)
    
    if len(sys.argv) > 2:
        port = sys.argv[1]
        baud = sys.argv[2]
    elif len(sys.argv) > 1:
        port = sys.argv[1]
        baud = 1000000
    else:
        # Use our default for the Mac Mini
        port = '/dev/tty.usbserial-A9005MZc'
        baud = 1000000

    try:
        ax12 = rio.AX12_IO(port, baud)
    except rio.RIOSerialError, e:
        print e.message, e.port, ' at ', e.baudrate
        sys.exit(1)
    
    # put port and baudrate on param server
    rospy.set_param('robot/ax12/port', port)
    rospy.set_param('robot/ax12/baudrate', baud)
    
    print 'Pinging motors...'
    
    motors = []
    for i in xrange(1, MAX_IDS_TO_PING):
        result = ax12.ping(i)
        if not result == []:
            motors.append(i)

    if motors:
        print 'Found motors: ' + str(motors)
    else:
        print'No motors found.'
  
    rospy.set_param('robot/ax12/motors', motors)
  
    print '\tComplete.'
    
    print 'Getting motor angle limits...'
    
    for i in motors:
        angles = ax12.get_min_max_angle_limits(i)
        rospy.set_param('robot/ax12/motor/%d/minAngle' %i, angles['min'])
        rospy.set_param('robot/ax12/motor/%d/maxAngle' %i, angles['max'])

    print '\tComplete.'

    print 'Verifying Param Server'
    
    if rospy.has_param('robot/ax12/motors'):
        value = rospy.get_param('robot/ax12/motors', [])
        for i in value:
            val = rospy.get_param('robot/ax12/motor/%d/minAngle' %i, 0)
            val2 = rospy.get_param('robot/ax12/motor/%d/maxAngle' %i, 0)
            print 'Id: %d, Min: %d, Max: %d' %(i, val, val2)
    else:
        print 'Error'

    print '\nSystem Initialized.\n'