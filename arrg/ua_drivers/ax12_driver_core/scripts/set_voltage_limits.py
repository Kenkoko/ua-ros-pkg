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
roslib.load_manifest('ax12_driver_core')

import sys
from optparse import OptionParser
from ax12_driver_core import ax12_io

if __name__ == '__main__':
    usage_msg = 'Usage: %prog [options] ID MIN_VOLTAGE MAX_VOLTAGE'
    desc_msg = 'Sets the minimum and maximum voltage limits of specified Dynamixel servo motor.'
    epi_msg = 'Example: %s --port=/dev/ttyUSB1 --baud=57600 1 10 16' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 4:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    motor_id = int(args[1])
    min_voltage = int(args[2])
    max_voltage = int(args[3])

    try:
        aio = ax12_io.AX12_IO(port, baudrate)
    except ax12_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        print 'Setting min to %d and max to %d:' % (min_voltage, max_voltage)
        print '%d ...' %motor_id ,
        if aio.ping(motor_id):
            aio.set_min_max_voltage_limits(motor_id, min_voltage, max_voltage)
            print 'done'
        else:
            print 'error'

