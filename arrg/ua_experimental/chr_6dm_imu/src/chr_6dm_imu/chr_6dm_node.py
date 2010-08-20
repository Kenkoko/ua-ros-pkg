#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Antons Rebguns. All rights reserved.
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
roslib.load_manifest('chr_6dm_imu')

import rospy

from chr_6dm_io import CHR6dmIMU
from sensor_msgs.msg import Imu
from tf import transformations as tft
import tf

class CHR6dmNode():
    def __init__(self):
        port_name = rospy.get_param('~port_name', '/dev/ttyUSB0')
        mode = rospy.get_param('~mode', 'streaming')
        data_rate = rospy.get_param('~data_rate', 100)
        frame_id = rospy.get_param('frame_id', 'imu')
        
        self.imu = CHR6dmIMU(port_name)
        self.imu.enable_accel_angrate_orientation()
        
        if mode == 'streaming':
            self.imu.set_broadcast_mode(data_rate)
        elif mode == 'polled':
            self.imu.set_silent_mode()
            
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = frame_id
        self.imu_data_pub = rospy.Publisher('imu/data', Imu)
        
    def publish_data(self):
        data = self.imu.read_accel_angrate_orientation()
        if not data: return
        print "RECEIVED DATA"
        print data
        ori = tft.quaternion_from_euler(data['roll'], data['pitch'], data['yaw'])
        print "ori = " + str(ori)
        
        self.imu_msg.orientation.x = ori[0]
        self.imu_msg.orientation.y = ori[1]
        self.imu_msg.orientation.z = ori[2]
        self.imu_msg.orientation.w = ori[3]
        print "orientation = " + str(self.imu_msg.orientation)
        
        self.imu_msg.angular_velocity.x = data['roll_rate']
        self.imu_msg.angular_velocity.y = data['pitch_rate']
        self.imu_msg.angular_velocity.z = data['yaw_rate']
        print "velocity = " + str(self.imu_msg.angular_velocity)

        self.imu_msg.linear_acceleration.x = data['accel_x']
        self.imu_msg.linear_acceleration.y = data['accel_y']
        self.imu_msg.linear_acceleration.z = data['accel_z']
        print "accel = " + str(self.imu_msg.linear_acceleration)

        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_data_pub.publish(self.imu_msg)
        
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 1), ori, self.imu_msg.header.stamp, self.imu_msg.header.frame_id, '/map')
        
    def shutdown(self):
        self.imu_data_pub.shutdown()
        self.imu.close()

if __name__ == '__main__':
    try:
        rospy.init_node('imu_node', anonymous=True)
        imu_node = CHR6dmNode()
        while not rospy.is_shutdown():
            imu_node.publish_data()
    except rospy.ROSInterruptException:
        imu_node.shutdown()

