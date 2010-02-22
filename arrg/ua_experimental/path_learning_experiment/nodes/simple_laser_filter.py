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

PKG = 'path_learning_experiment'

import roslib; roslib.load_manifest(PKG)

import time
from threading import Thread

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

class SimpleLaserFilter():
    def __init__(self):
        self.distance_pub = rospy.Publisher('front_distance', Float64)
        self.average_pub = rospy.Publisher('average_distance', Float64)
        rospy.init_node('simple_laser_filter', anonymous=True)
        rospy.Subscriber('tilt_scan', LaserScan, self.filter_scan)

    def filter_scan(self, scan):
        num_scans = len(scan.ranges)
        center_scan_index = (num_scans + 1) / 2
        center_range = scan.ranges[center_scan_index]
        self.distance_pub.publish(center_range)
        sum_dist = 0        
        for dist in scan.ranges:
            sum_dist += dist
        avg_dist = sum_dist / num_scans 
        self.average_pub.publish(avg_dist)

if __name__ == '__main__':
    try:
        simple_filter = SimpleLaserFilter()
        rospy.spin()
    except rospy.ROSInterruptException: pass

