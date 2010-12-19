#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_fft')
import rospy
from scipy.fftpack import fft, fftshift

import matplotlib
matplotlib.use('WXAgg')
from pylab import *

from ua_audio_msgs.msg import TransformedStream

class plot_fft(object):

    def __init__(self):
        self.fig = figure(1)
        ion()
        self.minpower = 1000000000
        self.maxpower = -1000000000

        rospy.init_node('plot_fft', anonymous=True)
        self.sub = rospy.Subscriber('transform_audio', TransformedStream, self.callback)

    def callback(self,data):
        #rospy.loginfo(rospy.get_name()+": I received %d datapoints.", data.num_points)
        freqArray = arange(0, data.num_points, 1.0) * (data.orig_rate/data.num_points);
        self.fig.clear()
        logpower = 10*log10(data.stream)
        self.minpower = min((self.minpower, min(logpower)))
        self.maxpower = max((self.maxpower, max(logpower)))
        plot(freqArray/1000, logpower, color='k')
        xlabel('Frequency (kHz)')
        ylabel('Power (dB)')
        ylim(self.minpower, self.maxpower)
        draw()
        show()


if __name__ == '__main__':
    A = plot_fft()
    rospy.spin()
