#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_fft')
import rospy
import threading
import sys
from scipy.fftpack import fft, fftshift

import matplotlib
matplotlib.use('WXAgg')
from pylab import *

from ua_audio_msgs.msg import AudioRawStream
from ua_audio_msgs.msg import TransformedStream

class audio_fft(object):

    def __init__(self):
        rospy.init_node('audio_fft', anonymous=True)
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber("audio_capture/audio", AudioRawStream, self.callback)
        self.pub = rospy.Publisher("fft_audio",TransformedStream)

        #Making sense of command-line arguments and/or using defaults
        self.wsz = 5
        self.ovr = 1
        if len(sys.argv) >= 2:
            self.wsz = int(sys.argv[1])
        if len(sys.argv) >= 3:
            self.ovr = int(sys.argv[2])
        if self.wsz < 1:
            self.wsz = 1
        if self.ovr < 1:
            self.ovr = 1
        if self.ovr > self.wsz:
            self.ovr = self.wsz
        #End command-line stuff
        self.numSamples = 0

    def callback(self,data):
        self.lock.acquire()

        if self.numSamples == 0:
            self.data = data.samples
        else:
            self.data = self.data + data.samples

        self.numSamples = self.numSamples + 1

        if self.numSamples == self.wsz:
            n = len(self.data) 
            p = fft(self.data) # take the fourier transform 
            nUniquePts = ceil((n+1)/2.0)
            p = p[0:nUniquePts]
            p = abs(p)
            p = p / float(n) # scale by the number of points so that
                             # the magnitude does not depend on the length 
                             # of the signal or on its sampling frequency  
            p = p**2  # square it to get the power 
            # odd nfft excludes Nyquist point
            if n % 2 > 0: # we've got odd number of points fft
                p[1:len(p)] = p[1:len(p)] * 2
            else:
                p[1:len(p) -1] = p[1:len(p) - 1] * 2 # we've got even number of points fft
            rospy.loginfo(rospy.get_name()+": publishing %d datapoints.", len(p))
            self.data = self.data[(len(data.samples)*self.ovr):len(self.data)]
            self.numSamples = self.numSamples - self.ovr
            self.lock.release()
            self.pub.publish(p,data.num_channels,data.sample_rate,nUniquePts,self.wsz,self.ovr)
        else:
            self.lock.release()


if __name__ == '__main__':
    A = audio_fft()
    rospy.spin()
