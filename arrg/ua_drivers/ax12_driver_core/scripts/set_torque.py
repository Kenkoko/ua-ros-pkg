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

import roslib
roslib.load_manifest('ax12_driver_core')

import sys
from ax12_driver_core import ax12_io

ON_STRINGS = ['on', 'true', '1']
OFF_STRINGS = ['off', 'false', '0']

USAGE = """\
Usage: %(executable)s command motor_id <port> <baudrate>
Usage: %(executable)s command list_of_motor_ids <port> <baudrate>
	- list_of_motor_ids is of the form [id1, id2, ...]
	- valid commands to turn torque on are:
		%(on_cmds)s
	- valid commands to turn torque off are:
		%(off_cmds)s
	- commands are not case-sensitive
	Examples:
	- %(executable)s on [1,2,3]
	- %(executable)s on [1,2,3] /dev/ttyUSB0
	- %(executable)s on [1,2,3] /dev/ttyUSB0 1000000
""" % { 'executable' : sys.argv[0], 
		'on_cmds' : str(ON_STRINGS).strip('[]'),
		'off_cmds' : str(OFF_STRINGS).strip('[]') }

def usage(msg=None):
	""" If msg is provided, print to stderr. Then print USAGE and exit.
	"""
	if msg:
		print >> sys.stderr, 'ERROR: %s' %msg
	print USAGE
	sys.exit(1)

def parseCommand(cmdStr):
	""" Parse first argument. Return True|False if argument is valid.
	Calls usage() otherwise.
	"""
	cmdStr = cmdStr.lower()
	if cmdStr in ON_STRINGS:
		return True
	if cmdStr in OFF_STRINGS:
		return False
	usage('Invalid command "%s"' %cmdStr)

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
	""" Called to parse arguments. Returns a 4-tuple containing command, values, port, baudrate.
	Calls usage() whenever an error is encountered.
	"""
	if len(sys.argv) < 3:
		usage('Not enough arguments.')
		
	command = sys.argv[1]
	torque_on = parseCommand(command)
	
	try:
		motor_id = int(sys.argv[2])
	except ValueError:
		motor_id = None
	motor_ids = None
	if not motor_id:
		motor_ids = parseIntegerList(sys.argv[2])
	
	if not motor_id and not motor_ids:
		usage('Invalid argument for motor_id(s) %s' %sys.argv[2])
	
	# default values
	baud = 1000000
	port = '/dev/ttyUSB0'
	
	if len(sys.argv) > 3:
		arg3 = sys.argv[3]
		try:
			baud = int(arg3)
		except ValueError:
			port = arg3	
		if len(sys.argv) == 5:
			arg4 = sys.argv[4]
			try:
				baud = int(arg4)
			except:
				usage('Invalid baudrate "%s"' %sys.argv[4])
	
	return (torque_on, [motor_id], port, baud) if motor_id else (torque_on, motor_ids, port, baud)

if __name__ == '__main__':
	torque_on, values, port, baudrate = parseArguments()
	try:
		aio = ax12_io.AX12_IO(port, baudrate)
	except ax12_io.SerialOpenError, soe:
		print 'ERROR:', soe
	else:
		responses = 0
		print 'Turning torque %s for motors:' %(('off', 'on')[torque_on])
		for motor_id in values:
			print '%d ...' %motor_id ,
			if aio.ping(motor_id):
				responses += 1
				aio.set_torque_enabled(motor_id, torque_on)
				print 'done'
			else:
				print 'error'
		if responses == 0:
			print 'ERROR: None of the specified motors responded. Make sure to specify the correct baudrate.'
	