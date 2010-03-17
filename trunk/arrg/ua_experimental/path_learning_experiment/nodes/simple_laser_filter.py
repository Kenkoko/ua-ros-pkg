#!/usr/bin/env python
# Author: Daniel Hewlett

PKG = 'path_learning_experiment'

import roslib; roslib.load_manifest(PKG)
import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from path_learning_experiment.msg import DistanceInfo

import time
import numpy

class SimpleLaserFilter():
    def __init__(self):
        #self.distance_pub = rospy.Publisher('front_dist', Float64)
        #self.average_pub = rospy.Publisher('avg_dist', Float64)
        #self.min_pub = rospy.Publisher('min_dist', Float64)
        self.dist_pub = rospy.Publisher('dist_info', DistanceInfo)        

        rospy.init_node('simple_laser_filter', anonymous=True)

        rospy.Subscriber('tilt_scan', LaserScan, self.filter_scan)

    def filter_scan(self, scan):
        di = DistanceInfo()
        di.header = scan.header    
        di.min_dist = numpy.min(scan.ranges)
        num_scans = len(scan.ranges)
        center_scan_index = (num_scans + 1) / 2
        center_range = scan.ranges[center_scan_index]
        di.front_dist = center_range
        sum_dist = 0        
        for dist in scan.ranges:
            sum_dist += dist
        di.avg_dist = sum_dist / num_scans 
        self.dist_pub.publish(di)

if __name__ == '__main__':
    try:
        simple_filter = SimpleLaserFilter()
        rospy.spin()
    except rospy.ROSInterruptException: pass

