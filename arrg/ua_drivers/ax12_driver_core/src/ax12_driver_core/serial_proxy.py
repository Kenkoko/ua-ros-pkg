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
# Author: Cody Jorgensen
# Author: Cara Slutter
#

import roslib
roslib.load_manifest('ax12_driver_core')

import rospy

import ax12_io
from ax12_driver_core.msg import MotorState
from ax12_driver_core.msg import MotorStateList

import sys
from threading import Lock
from threading import Thread
from Queue import Queue

class SerialProxy():
    def __init__(self, port_name='/dev/ttyUSB0', baud_rate='1000000', min_motor_id=1, max_motor_id=25, update_rate=5):
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.min_motor_id = min_motor_id
        self.max_motor_id = max_motor_id
        self.update_rate = update_rate
        
        self.__packet_queue = Queue()
        self.__state_lock = Lock()
        
        self.motor_states_pub = rospy.Publisher('motor_states', MotorStateList)

    def connect(self):
        try:
            self.__serial_bus = ax12_io.AX12_IO(self.port_name, self.baud_rate)
            self.__find_motors()
        except ax12_io.SerialOpenError, e:
            rospy.logfatal(e.message)
            sys.exit(1)
            
        self.running = True
        Thread(target=self.__process_packet_queue).start()
        if self.update_rate > 0:
            Thread(target=self.__update_motor_states).start()

    def disconnect(self):
        self.running = False
        self.queue_new_packet('shutdown')

    def queue_new_packet(self, packet):
        self.__packet_queue.put_nowait(packet)

    def __find_motors(self):
        rospy.loginfo('Pinging motor IDs %d through %d...' % (self.min_motor_id, self.max_motor_id))
        
        self.motors = []
        for i in xrange(self.min_motor_id, self.max_motor_id):
            result = self.__serial_bus.ping(i)
            if result: self.motors.append(i)
            
        if self.motors:
            rospy.loginfo('Found motors with IDs: %s.' % str(self.motors))
        else:
            rospy.logfatal('No motors found, aborting.')
            sys.exit(1)
            
        rospy.set_param('ax12/connected_ids', self.motors)
        rospy.loginfo('Publishing motor angle limits to param server...')
        
        for i in self.motors:
            angles = self.__serial_bus.get_min_max_angle_limits(i)
            rospy.set_param('ax12/%d/minAngle' %i, angles['min'])
            rospy.set_param('ax12/%d/maxAngle' %i, angles['max'])
            
        rospy.loginfo('System initialized.')

    def __process_packet_queue(self):
        while self.running:
            # block until new packet is available
            packet = self.__packet_queue.get(True)
            if packet == 'shutdown': return
            self.__state_lock.acquire()
            try:
                self.__serial_bus.write_packet(packet)
            except ax12_io.FatalErrorCodeError, fece:
                rospy.logfatal(fece)
                signal_shutdown(fece)
            except ax12_io.NonfatalErrorCodeError, nfece:
                rospy.logwarn(nfece)
            except ax12_io.ChecksumError, cse:
                rospy.logwarn(cse)
            except ax12_io.DroppedPacketError, dpe:
                rospy.loginfo(dpe.message)
            finally:
                self.__state_lock.release()

    def __update_motor_states(self):
        rate = rospy.Rate(self.update_rate)
        while self.running:
            self.__state_lock.acquire()
            # get current state of all motors and publish to motor_states topic
            motor_states = []
            for i in self.motors:
                try:
                    state = self.__serial_bus.get_servo_feedback(i)
                    if state:
                        motor_states.append(MotorState(**state))
                        if ax12_io.exception: raise ax12_io.exception
                except ax12_io.FatalErrorCodeError, fece:
                    rospy.logfatal(fece)
                    signal_shutdown(fece)
                except ax12_io.NonfatalErrorCodeError, nfece:
                    rospy.logwarn(nfece)
                except ax12_io.ChecksumError, cse:
                    rospy.logwarn(cse)
                except ax12_io.DroppedPacketError, dpe:
                    rospy.loginfo(dpe.message)
            self.__state_lock.release()
                
            if motor_states:
                msl = MotorStateList()
                msl.header.stamp = rospy.Time.now()
                msl.motor_states = motor_states
                self.motor_states_pub.publish(msl)
                
            rate.sleep()

if __name__ == '__main__':
    try:
        serial_proxy = SerialProxy()
        serial_proxy.connect()
        rospy.spin()
        serial_proxy.disconnect()
    except rospy.ROSInterruptException: pass

