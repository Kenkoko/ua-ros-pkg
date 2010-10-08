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
    parser = OptionParser(usage='Usage: %prog [options] OLD_ID NEW_ID', description='Changes the unique ID of a Dynamixel servo motor.')
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 3:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    old_id = int(args[1])
    new_id = int(args[2])
    
    try:
        aio = ax12_io.AX12_IO(port, baudrate)
    except ax12_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        print 'Changing motor id from %d to %d...' %(old_id, new_id),
        if aio.ping(old_id):
            aio.set_servo_id(old_id, new_id)
            print ' done'
            print 'Verifying new id...' ,
            if aio.ping(new_id):
                print ' done'
            else:
                success = False
                print 'error'
                print 'The motor did not respond to a ping to its new id.'
        else:
            print 'error'
            print 'The specified motor did not respond to id %d. Make sure to specify the correct baudrate.' %old_id

