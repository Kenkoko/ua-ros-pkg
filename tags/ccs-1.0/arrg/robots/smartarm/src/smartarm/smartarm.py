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
roslib.load_manifest('smartarm')

import rospy
from ax12.msg import Move
from smartarm.msg import JointMove

class SmartArmJoint:

	def __init__(self, joint_id, master_id, inverse_id=None):
		self.jid = joint_id
		self.master = master_id
		self.inverse = inverse_id

	def servo_positions_for_angle(self, angleInDeg):
		if angleInDeg > 300 or angleInDeg < 0:
			raise ValueError('Angle must be between 0-300 degrees')
		a = {}
		a[self.master] = int(1023.0 / 300.0 * angleInDeg)
		if self.inverse is not None:
			a[self.inverse] = 1023 - a[self.master]
		return a

class SmartArm:

	def __init__(self):
		self.joints = {}
		numJoints = rospy.get_param('/smartarm/ax12/joint_count', 0)
		for i in xrange(numJoints):
			master_id = rospy.get_param('/smartarm/ax12/joint%d/master' %i, None)
			inverse_id = rospy.get_param('/smartarm/ax12/joint%d/inverse' %i, None)
			self.joints[i] = SmartArmJoint(i, master_id, inverse_id)
		self.publisher = rospy.Publisher('robot/ax12/moves', Move)

	def set_joint_to_angle(self, jointNum, angleInDeg):
		servo_positions = self.joints[jointNum].servo_positions_for_angle(angleInDeg)
		for sid in servo_positions:
			self.publisher.publish(sid, servo_positions[sid], 200)

	def handleInput(self, joint_move):
		self.set_joint_to_angle(joint_move.id, joint_move.angle)

if __name__ == '__main__':
	rospy.init_node('smartarm_joint_mapping', anonymous=False)
	sa = SmartArm()
	rospy.Subscriber('smartarm/ax12/interface', JointMove, sa.handleInput)
	print 'SmartArm is waiting for input.'
	rospy.spin()
