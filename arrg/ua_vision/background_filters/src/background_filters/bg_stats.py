#!/usr/bin/env python
import roslib
roslib.load_manifest('background_filters')

import rospy
import cv
import cv_bridge

from sensor_msgs.msg import Image
from background_filters.srv import GetBgStats

import numpy
import math

class BackgroundAverager:
    def __init__(self):
        rospy.init_node('background_averager', anonymous=True)
        rospy.Subscriber('image', Image, self.handle_image)
        rospy.Service('get_background_stats', GetBgStats, self.get_bg_stats)
        self.bg_num = rospy.get_param('~bg_num', 10)

        self.bridge = cv_bridge.CvBridge()
        #self.win1 = cv.NamedWindow('win1')
        #self.win2 = cv.NamedWindow('win2')
        self.counter = 0
        self.have_ave_bg = False
        self.numpy_bgs = []
        self.covariances = []
        self.averages = []
        self.determinants = []
        self.std_devs = []

    def handle_image(self, msg):
        if self.counter < self.bg_num:
            cv_image = self.bridge.imgmsg_to_cv(msg, "bgr8")
            self.numpy_bgs.append(numpy.asarray(cv_image))
            self.counter += 1
            return

        if not self.have_ave_bg:
            self.have_ave_bg = True
            rospy.loginfo('Collected samples for background image averaging')

            height = self.numpy_bgs[0].shape[0]
            width = self.numpy_bgs[0].shape[1]
            n_channels = self.numpy_bgs[0].shape[2]
            depth = cv.IPL_DEPTH_8U
            size = self.numpy_bgs[0].size
            rospy.loginfo('height = %d, width = %d, n_channels = %d, depth = %d, size = %d' % (height, width, n_channels, depth, size))

            cov_mat = cv.CreateMat(3, 3, cv.CV_32FC1)
            ave_arr = cv.CreateMat(1, 3, cv.CV_32FC1)

            # for each pixel in the image
            for i in xrange(0, size, 3):
                vects = []
                # for each image that we sampled
                for img in self.numpy_bgs:
                    mat = cv.CreateMatHeader(1, 3, cv.CV_8UC1)
                    cv.SetData(mat, img.take([i, i+1, i+2]), 3)
                    vects.append(mat)
                cv.CalcCovarMatrix(vects, cov_mat, ave_arr, cv.CV_COVAR_NORMAL)
                self.determinants.append(cv.Det(cov_mat))
                self.covariances.extend(numpy.asarray(cov_mat, dtype=numpy.uint8).ravel())
                ave_np = numpy.asarray(ave_arr, dtype=numpy.uint8).ravel()
                self.averages.extend(ave_np)
                sdb = sdg = sdr = 0.0
                for img in self.numpy_bgs:
                    (b, g, r) = img.take([i, i+1, i+2])
                    sdb += pow(b - ave_np[0], 2.0)
                    sdg += pow(g - ave_np[1], 2.0)
                    sdr += pow(r - ave_np[2], 2.0)
                self.std_devs.append(math.sqrt(sdb / (self.bg_num - 1.0)))
                self.std_devs.append(math.sqrt(sdg / (self.bg_num - 1.0)))
                self.std_devs.append(math.sqrt(sdr / (self.bg_num - 1.0)))

            ave_numpy = numpy.array(self.averages, dtype=numpy.uint8)
            self.ave_img = cv.CreateImageHeader((width, height), cv.IPL_DEPTH_8U, 3)
            cv.SetData(self.ave_img, ave_numpy, width * 3)

            #cv.ShowImage('win1', self.ave_img)
            #cv.ShowImage('win1', self.numpy_bgs[0])
            #cv.ShowImage('win2', cv.fromarray(self.numpy_bgs[0]))
            #cv.WaitKey(1000)

            rospy.loginfo('Computed average background image from samples')

    def get_bg_stats(self, req):
        res = GetBgStats()
        res.average_background = self.bridge.cv_toimgmsg(self.ave_img)
        res.covariance_matrix = self.covariances
        res.covariance_matrix_dets = self.determinants
        res.standard_deviations = self.std_devs
        return res

if __name__ == '__main__':
    b = BackgroundAverager()
    rospy.spin()

