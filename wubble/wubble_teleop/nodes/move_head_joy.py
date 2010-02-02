#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Daniel Hewlett
# Author: Antons Rebguns

PKG = 'wubble_teleop'

import roslib; roslib.load_manifest(PKG)

import time
from threading import Thread

import rospy
from joy.msg import Joy
#from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

class MoveHeadXbox():
    def __init__(self):
        self.is_running = True
        self.head_pan = 0.0
        self.head_tilt = 0.0
        self.joy_data = None
        self.joint_traj = JointTrajectory()
        self.joint_traj.joint_names = ['head_pan_joint', 'head_tilt_joint'] #'head_tilt_joint']
        self.jtp = JointTrajectoryPoint()
        self.jtp.positions = [0.0, 0.0]
        self.jtp.velocities = [0.0, 0.0]
        self.jtp.accelerations = [0.0, 0.0]
        self.jtp.time_from_start = 1.0
        self.joint_traj.points = [self.jtp]
        
        self.head_angles_pub = rospy.Publisher('head_traj_controller/command', JointTrajectory)
        rospy.init_node('move_head_joy', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.read_joystick_data)
        rospy.Subscriber('head_traj_controller/state', JointTrajectoryControllerState, self.read_current)

    def read_joystick_data(self, data):
        self.joy_data = data

    def bound(self, number):
        if (number < -1): return -1
        elif (number > 1): return 1
        else: return number 

    def set_head_position(self, delta_pan, delta_tilt):
        self.head_pan += self.bound(delta_pan * 0.08)
        self.head_tilt += self.bound(delta_tilt * 0.1)
        self.jtp.positions = [self.head_pan, self.head_tilt]

    def reset_head_position(self):
        self.jtp.positions = [0.0, 0.0]
        self.head_pan = 0.0
        self.head_tilt = 0.0

    def read_current(self, data):
        pass
        #self.head_pan = data.actual.positions[0]
        #self.head_tilt = data.actual.positions[1]

    def update_head_position(self):
        while self.is_running:
            if self.joy_data:
                if self.joy_data.buttons[4]: 
                    self.reset_head_position()
                    print "Attempting to reset head position"
                else: 
                    self.set_head_position(self.joy_data.axes[2], -1.0 * self.joy_data.axes[3])
            
                if self.joy_data.buttons[7]:
                    print self.joint_traj
                    self.head_angles_pub.publish(self.joint_traj)
            
            time.sleep(0.1)


if __name__ == '__main__':
    try:
        move_head = MoveHeadXbox()
        t = Thread(target=move_head.update_head_position)
        t.start()
        rospy.spin()
        move_head.is_running = False
        t.join()
        move_head.reset_head_position()
    except rospy.ROSInterruptException: pass

