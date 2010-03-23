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
Usage: %(executable)s <motor_id>
Usage: %(executable)s <list_of_motor_ids>
    - list_of_motor_ids is of the form [id1, id2, ...]
    Examples:
    - %(executable)s
    - %(executable)s [1,2,3]
""" % { 'executable' : sys.argv[0], }

def usage(msg=None):
    """ If msg is provided, print to stderr. Then print USAGE and exit.
    """
    if msg:
        print >> sys.stderr, 'ERROR: %s' %msg
    print USAGE
    sys.exit(1)

def parseIntegerList(listStr):
    """ Takes a string representation of a list of integers and converts it
    into a list of ints. Returns an empty list if parsing fails.
    """
    listStr = listStr.strip()
    if listStr[0] != '[' or listStr[-1] != ']':
        return []
    strIntList = listStr[1:-1].strip(',').split(',')
    try:
        return map(int, strIntList)
    except:
        return []

def parseArguments():
    """ Called to parse arguments. Returns a 3-tuple containing values, port, baudrate.
    Calls usage() whenever an error is encountered.
    """
    if len(sys.argv) < 2:
        usage('Not enough arguments.')

    if len(sys.argv) > 4:
        usage('Too many arguments.')
        
    try:
        motor_id = int(sys.argv[1])
    except ValueError:
        motor_id = None
    motor_ids = None
    if not motor_id:
        motor_ids = parseIntegerList(sys.argv[1])

    if not motor_id and not motor_ids:
        usage('Invalid argument for motor_id(s): %s' %sys.argv[1])

    # default values
    baud = 1000000
    port = '/dev/ttyUSB0'

    if len(sys.argv) > 2:
        arg2 = sys.argv[2]
        try:
            baud = int(arg2)
        except ValueError:
            port = arg2    
        if len(sys.argv) == 4:
            arg3 = sys.argv[3]
            try:
                baud = int(arg3)
            except:
                usage('Invalid baudrate "%s"' %sys.argv[3])

    return ([motor_id], port, baud) if motor_id else (motor_ids, port, baud)

def print_data(values):
    ''' Takes a dictionary with all the motor values and does a formatted print.
    '''
    if values['freespin']:
        print '''\
    Motor %(id)d is connected:
        Freespin: True
        Current Speed ----------- %(speed)d
        Current Temperature ----- %(temperature)d%(degree_symbol)sC
        Current Voltage --------- %(voltage).1fv
        Current Load ------------ %(load)d
        Moving ------------------ %(moving)s
''' %values
    else:
        print '''\
    Motor %(id)d is connected:
        Freespin: False
        Min Angle --------------- %(min)d
        Max Angle --------------- %(max)d
        Current Position -------- %(position)d
        Current Speed ----------- %(speed)d
        Current Temperature ----- %(temperature)d%(degree_symbol)sC
        Current Voltage --------- %(voltage).1fv
        Current Load ------------ %(load)d
        Moving ------------------ %(moving)s
''' %values
    

if __name__ == '__main__':
    motor_ids, port, baudrate  = parseArguments()
    try:
        aio = ax12_io.AX12_IO(port, baudrate)
    except ax12_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        responses = 0
        print 'Pinging motors:'
        for motor_id in motor_ids:
            print '%d ...' %motor_id ,
            p = aio.ping(motor_id)
            if p:
                responses += 1
                values = aio.get_servo_feedback(motor_id)
                angles = aio.get_min_max_angle_limits(motor_id) 
                values['degree_symbol'] = u"\u00B0"
                values['min'] = angles['min']
                values['max'] = angles['max']
                values['voltage'] = values['voltage'] / 10.0
                values['moving'] = str(values['moving'])
                print 'done'
                if angles['max'] == 0 and angles['min'] == 0:
                    values['freespin'] = True
                else:
                    values['freespin'] = False
                print_data(values)
            else:
                print 'error'
        if responses == 0:
            print 'ERROR: None of the specified motors responded. Make sure to specify the correct baudrate.'