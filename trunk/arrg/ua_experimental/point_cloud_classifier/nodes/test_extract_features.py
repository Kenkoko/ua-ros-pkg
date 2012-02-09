#!/usr/bin/env python

import sys

import roslib; roslib.load_manifest('point_cloud_classifier')
import rospy

from point_cloud_classifier.srv import ExtractFeatures


if __name__ == '__main__':
    rospy.init_node('test_extract_features', anonymous=True)
    extract_features = rospy.ServiceProxy('extract_features', ExtractFeatures)
    
    fname = '/home/anton/Downloads/rgbd-dataset/apple/apple_1/apple_1_1_1.pcd'
    
    try:
        res = extract_features(fname)
    except Exception as e:
        rospy.logerr('extract_features service call failed: %s' % str(e))
        exit(0)
    
    rospy.loginfo('num_features: %d, feature_size: %d, num_bins: %d' % (res.num_features, res.feature_size, res.num_bins))
