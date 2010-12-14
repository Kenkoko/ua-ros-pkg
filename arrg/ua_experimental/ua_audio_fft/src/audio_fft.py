#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_transform')
import rospy
import sys
import traceback
import getopt
from scipy.fftpack import fft, fftshift

from ua_audio_msgs.msg import AudioRawStream
from ua_audio_msgs.msg import TransformedStream

class audio_fft(object):

    def __init__(self,argv):
        rospy.init_node('audio_transform', anonymous=True)
        self.sub = rospy.Subscriber("audio_capture/audio", AudioRawStream, self.callback)
        self.pub = rospy.Publisher("transform_audio",TransformedStream)
        self.parse_cline(argv)
        self.first_time = True

    def parse_cline(self,argv):
        self.wszp = 0.25
        self.ovrp = 0.125
        self.pwr = True
        try:
            opts, args = getopt.getopt(argv, "w:o:p:", ["window-size=","overlap=","powers_of_two="])
        except:
            traceback.print_exc()
            rospy.loginfo(rospy.get_name()+": Command line fail")
            sys.exit(2)
        for opt, arg in opts:
            if opt in ("-w","--window-size"):
                rospy.loginfo(rospy.get_name()+": window-size="+arg)
                self.wszp = float(arg)
            elif opt in ("-o","--overlap"):
                rospy.loginfo(rospy.get_name()+": overlap="+arg)
                self.ovrp = float(arg)
            elif opt in ("-p","--powers_of_two"):
                if arg=="False":
                    self.pwr = False
                rospy.loginfo(rospy.get_name()+": powers_of_two="+str(self.pwr))
        if self.wszp <= 0:
            self.wszp = 0.25
        if self.ovrp <= 0:
            self.ovrp = 0.125
        if self.ovrp > self.wszp:
            self.ovrp = self.wszp

    def init_freq(self,data):
        self.first_time = False
        self.wsz = int(data.sample_rate * self.wszp)
        self.ovr = int(data.sample_rate * self.ovrp)
        if self.pwr:
            lo_pwr = 1
            hi_pwr = 2
            while hi_pwr < self.wsz:
                lo_pwr = hi_pwr
                hi_pwr = hi_pwr << 1
            if self.wsz - lo_pwr < hi_pwr - self.wsz:
                self.wsz = lo_pwr
            else:
                self.wsz = hi_pwr
            lo_pwr = 1
            hi_pwr = 2
            while hi_pwr < self.ovr:
                lo_pwr = hi_pwr
                hi_pwr = hi_pwr << 1
            if self.ovr - lo_pwr < hi_pwr - self.ovr:
                self.ovr = lo_pwr
            else:
                self.ovr = hi_pwr
        rospy.loginfo(rospy.get_name()+": From frequency %d, using window-size %d and overlap %d.",data.sample_rate,self.wsz,self.ovr)
        self.nUniquePts = (self.wsz+1)/2 + (self.wsz+1)%2

    def callback(self,data):
        
        if(self.first_time):
            self.data = data.samples
            self.init_freq(data)
        else:
            self.data = self.data + data.samples

        while len(self.data) >= self.wsz:
            p = fft(self.data[0:self.wsz-1]) # take the fourier transform 
            p = p[0:self.nUniquePts]
            p = abs(p)
            p = p / float(self.wsz) # scale by the number of points so that
                             # the magnitude does not depend on the length 
                             # of the signal or on its sampling frequency  
            p = p**2  # square it to get the power 
            # odd nfft excludes Nyquist point
            if self.wsz % 2 > 0: # we've got odd number of points fft
                p[1:len(p)] = p[1:len(p)] * 2
            else:
                p[1:len(p) -1] = p[1:len(p) - 1] * 2 # we've got even number of points fft
            rospy.loginfo(rospy.get_name()+": publishing %d datapoints.", len(p))
            self.data = self.data[self.ovr:len(self.data)]
            self.pub.publish(p,1,data.num_channels,data.sample_rate,self.nUniquePts,self.wsz,self.ovr)
            #Currently the '1' indicates that it is fft transform. Other transforms can be indicated by other integers.

if __name__ == '__main__':
    A = audio_fft(sys.argv[1:])
    rospy.spin()
