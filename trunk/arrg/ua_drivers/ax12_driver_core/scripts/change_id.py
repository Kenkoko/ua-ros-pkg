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
from ax12_driver_core.ax12_const import AX_ID

USAGE = """\
Usage: %(executable)s old_motor_id new_motor_id <port> <baudrate>
	Examples:
	- %(executable)s 1 8
	- %(executable)s 1 8 /dev/ttyUSB0
	- %(executable)s 1 8 /dev/ttyUSB0 1000000
""" % { 'executable' : sys.argv[0], }

def usage(msg=None):
	""" If msg is provided, print to stderr. Then print USAGE and exit.
	"""
	if msg:
		print >> sys.stderr, 'ERROR: %s' %msg
	print USAGE
	sys.exit(1)

def parseArguments():
	""" Called to parse arguments. Returns a 4-tuple containing old_id, new_id, port, baudrate.
	Calls usage() whenever an error is encountered.
	"""
	if len(sys.argv) < 3:
		usage('Not enough arguments.')
	
	try:
		old_id = int(sys.argv[1])
	except ValueError:
		usage('Invalid old_motor_id "%s"' %sys.argv[1])
		
	try:
		new_id = int(sys.argv[2])
	except ValueError:
		usage('Invalid new_motor_id "%s"' %sys.argv[2])
	motor_ids = None
	
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
	
	return (old_id, new_id, port, baud)

if __name__ == '__main__':
	old_id, new_id, port, baudrate = parseArguments()
	try:
		aio = ax12_io.AX12_IO(port, baudrate)
	except ax12_io.SerialOpenError, soe:
		print 'ERROR:', soe
	else:
		print 'Changing motor id from %d to %d' %(old_id, new_id)
		print 'Changing ...' ,
		if aio.ping(old_id):
			aio.write_to_servo(old_id, AX_ID, (new_id,))
			print 'done'
			print 'Verifying new id ...' ,
			if aio.ping(new_id):
				print 'done'
			else:
				success = False
				print 'error'
				print 'The motor did not respond to a ping to its new id.'
		else:
			print 'error'
			print 'The specified motor did not respond to id %d. Make sure to specify the correct baudrate.' %old_id