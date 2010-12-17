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
#

import roslib
roslib.load_manifest('ax12_driver_core')

import sys
from optparse import OptionParser
from ax12_driver_core import ax12_io

if __name__ == '__main__':
    usage_msg = 'Usage: %prog [options] MOTOR_IDs'
    desc_msg = 'Sets various configuration options of specified Dynamixel servo motor.'
    epi_msg = 'Example: %s --port=/dev/ttyUSB1 --baud=57600 --baud-rate=1 --return-delay=1 5 9 23' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type='int', default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
    parser.add_option('-r', '--baud-rate', type='int', metavar='RATE', dest='baud_rate',
                      help='set servo motor communication speed')
    parser.add_option('-d', '--return-delay', type='int', metavar='DELAY', dest='return_delay',
                      help='set servo motor return packet delay time')
    parser.add_option('--cw-angle-limit', type='int', metavar='CW_ANGLE', dest='cw_angle_limit',
                      help='set servo motor CW angle limit')
    parser.add_option('--ccw-angle-limit', type='int', metavar='CCW_ANGLE', dest='ccw_angle_limit',
                      help='set servo motor CCW angle limit')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 2:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    motor_ids = args[1:]

    try:
        aio = ax12_io.AX12_IO(port, baudrate)
    except ax12_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        for motor_id in motor_ids:
            motor_id = int(motor_id)
            print 'Configuring Dynamixel motor with ID %d' % motor_id
            if aio.ping(motor_id):
                if options.baud_rate:
                    print 'Setting baud rate to %d bps' % int(2000000.0/(options.baud_rate + 1))
                    aio.set_servo_baud_rate(motor_id, options.baud_rate)
                if options.return_delay:
                    print 'Setting return delay time to %d us' % options.return_delay * 2
                    aio.set_servo_return_delay_time(motor_id, options.return_delay)
                if options.cw_angle_limit:
                    print 'Setting CW angle limit to %d' % options.cw_angle_limit
                    aio.set_servo_angle_limit_cw(motor_id, options.cw_angle_limit)
                if options.ccw_angle_limit:
                    print 'Setting CCW angle limit to %d' % options.ccw_angle_limit
                    aio.set_servo_angle_limit_ccw(motor_id, options.ccw_angle_limit)
                print 'done'
            else:
                print 'Unable to connect to Dynamixel motor with ID %d' % motor_id

