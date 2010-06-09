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
#

import roslib
roslib.load_manifest('ax12_driver_core')

import sys
from ax12_driver_core import ax12_io

USAGE = """\
Usage: %(executable)s motor_id min_voltage max_voltage <port> <baudrate>
    Set both min and max angles to 0 to enable "freespin".
    Examples:
    - %(executable)s 1 0 1023
    - %(executable)s 1 200 800
    - %(executable)s 1 0 0
    - %(executable)s 1 0 0 /dev/ttyUSB0
    - %(executable)s 1 0 0 /dev/ttyUSB0 1000000
""" % { 'executable' : sys.argv[0], }

def usage(msg=None):
    """ If msg is provided, print to stderr. Then print USAGE and exit.
    """
    if msg:
        print >> sys.stderr, 'ERROR: %s' %msg
    print USAGE
    sys.exit(1)

def parseArguments():
    """ Called to parse arguments. Returns a 5-tuple containing motor_id, min_voltage,max_voltage, port, baudrate.
    Calls usage() whenever an error is encountered.
    """
    if len(sys.argv) < 4:
        usage('Not enough arguments.')

    try:
        motor_id = int(sys.argv[1])
    except ValueError:
        usage('Invalid motor id: %s' %sys.argv[1])

    try:
        min_voltage = int(sys.argv[2])
    except ValueError:
        usage('Invalid min angle: %s' %sys.argv[1])
            
    try:
       max_voltage = int(sys.argv[3])
    except ValueError:
        usage('Invalid max angle: %s' %sys.argv[1])

    # default values
    baud = 1000000
    port = '/dev/ttyUSB0'

    if len(sys.argv) > 4:
        arg4 = sys.argv[4]
        try:
            baud = int(arg4)
        except ValueError:
            port = arg4
        if len(sys.argv) == 6:
            arg5 = sys.argv[5]
            try:
                baud = int(arg5)
            except:
                usage('Invalid baudrate "%s"' %sys.argv[5])

    return (motor_id, min_voltage,max_voltage, port, baud)

if __name__ == '__main__':
    motor_id, min_voltage,max_voltage, port, baudrate  = parseArguments()
    try:
        aio = ax12_io.AX12_IO(port, baudrate)
    except ax12_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        print 'Setting min to %d and max to %d:' %(min_voltage,max_voltage)
        print '%d ...' %motor_id ,
        if aio.ping(motor_id):
            aio.set_min_max_voltage_limits(motor_id, min_voltage,max_voltage)
            print 'done'
        else:
            print 'error'
