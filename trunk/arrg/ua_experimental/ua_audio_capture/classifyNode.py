#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_capture')
import sys
import pickle
import math
import os
from array import array
from operator import itemgetter

import rospy

import pylab as pl
import numpy as np
import matplotlib
from scipy.ndimage.filters import gaussian_filter1d

import nwalign as nw
from scikits.learn.cluster import SelfOrganizingMap
from Bio import kNN

############################################################################
#
# ROS node for SOM/K nearest neighbors classification of audio 
#
############################################################################

__author__ = 'Antons Rebguns, antons@email.arizona.edu; Daniel Ford, dford@email.arizona.edu'

from ua_audio_capture.srv import *

class classifyNode():

    def __init__(self):
           
        # init node and service
        rospy.init_node('classifyNode')
        request = rospy.Service('classify', classify, self._handleSvcRequest)
        print "\nReady for audio-based classification..."

        # load previously trained SOM and knn_model from pkl files
        self.som = pickle.load(open('/tmp/som.pkl'))
        self.knn_model = pickle.load(open('/tmp/knn_model.pkl'))

    # callback that handles actual work
    def _handleSvcRequest(self, req):
        
        # calculate FFT of raw sound
        sound_fft = self.calculate_fft(req.rawAudio)

        # do classification, return object names and beliefs
        mostLikely, beliefDict = self.classify(sound_fft)

        print beliefDict.keys()
        print beliefDict.values()
        print "\n"

        # send names and beliefs to requesting node
        return classifyResponse(beliefDict.keys(), beliefDict.values())

    # classify sound based on SOM and K nearest neighbors
    def classify(self, sound_fft):
        """
        Classify a new sound. Takes in a sound FFT, a SOM model and kNN model.
        Returns a tuple containing a highest probability label and a dictionary of
        all label probabilities.
        """
        sequence = []
        
        for col in range(sound_fft.shape[1]):
            sequence.append(self.som.bmu(sound_fft[:,col]))
            
        weights = kNN.calculate(self.knn_model,
                                self.stringify_sequence(sequence),
                                weight_fn=self.knn_weight_fn,
                                distance_fn=self.sound_seq_distance_str)
                              
        sum_weights = float(sum(weights.values()))
        
        most_class = None
        most_weight = None
        
        for klass, weight in weights.items():
            weights[klass] = weight / sum_weights
            
            if most_class is None or weight > most_weight:
                most_class = klass
                most_weight = weight
                
        return most_class, weights

    # FFT calculation function
    def calculate_fft(self,rawAudio):
        """
        Given raw audio data, computes and returns FFTs
        """
        ####### Parameters ######
        sampling_rate = 44100
        fft_n = 512
        fft_overlap = 256
        fft_time_after_peak = 2.0
        fft_freq_bins = 33
        #########################
        
        #processed_ffts = []
        
        print 'Calculating start time...'
        start_time = self.find_sound_start(rawAudio)
        
        print 'Calculating end time...'
        end_time = self.find_sound_end(rawAudio, start_time)
        
        # chop silence off beginning and end of sound
        s = np.asarray(rawAudio)
        start = int(start_time)
        end = start + int(fft_time_after_peak * sampling_rate)
        s = s[start:end]
        
        Pxx,freqs,t = matplotlib.mlab.specgram(s, NFFT=fft_n, Fs=sampling_rate, noverlap=fft_overlap)
        Pxx = 20 * np.log10(Pxx)
        freqs /= 1000.0
        
        l = np.linspace(0, Pxx.shape[0], fft_freq_bins+1)
        fft_binned = np.empty((fft_freq_bins,Pxx.shape[1]))
        
        for i in range(len(l)-1):
            lis = int(l[i])
            lie = int(l[i+1])
            t = Pxx[lis:lie,:]
            fft_binned[i] = t.mean(axis=0)
            
        #processed_fft.append(fft_binned)
        
        return fft_binned

    def knn_weight_fn(self,x, y):
        #dist = sound_seq_distance_str(x, y)
        return 1#math.exp(dist)

    def rebin_time_fixed_width(self, s, w):
        # filter first
        s = gaussian_filter1d(s, 10, mode='constant')

        num_intervals = int(s.shape[0] / w + 1)
        result = []
        #print 'len(s)', len(s)

        for i in range(num_intervals-1):
            start = i*w
            end = start + w
            m = s[start:end].mean()
            result.append(m)
            
        return np.asarray(result)

    # convert tuples to strings for matching
    def stringify_sequence(self,seq):
        """
        input  = [(1,2),(2,3) ...]
        output = 'ab....'
        """
        map_side = 6
        result = []
        for t in seq:
            idx = t[0] * map_side + t[1]
            ascii_idx = idx + 48
            result.append(chr(ascii_idx))
        return ''.join(result)

    # compares two strings and returns the distance between them
    # requires som.costs file to be in /tmp
    def sound_seq_distance_str(self,seq1_str, seq2_str):
        seq1_str = np.asanyarray(seq1_str)
        seq2_str = np.asanyarray(seq2_str)
        
        align = nw.global_align(seq1_str.tostring(), seq2_str.tostring(), gap_open=0, gap_extend=-5, matrix='/tmp/som.costs')
        len1 = len(seq1_str.tostring())
        len2 = len(seq2_str.tostring())
        return (-nw.score_alignment(*align, gap_open=0, gap_extend=-5, matrix='/tmp/som.costs'))/(len1+len2+0.0)

    # find the start of a sound
    def find_sound_start(self,rawAudio):
        window = int(0.125*44100)

        def index(seq, f):
            """Return the index of the first item in seq where f(item) == True."""
            return next((i for i in xrange(len(seq)) if f(seq[i])), None)

        counter = 0
        total = 0
        
        s = np.asarray(rawAudio)
        s = np.abs(s)
        s = self.rebin_time_fixed_width(s, window)
        maximum = max(s)
        start = index(s, (lambda x: x > maximum / 2.0))
        
        if start is not None:
            start *= window
            start = max(0, start - 0.1*44100)
            total += start
            counter += 1
            start_time = start
        else:
            start_time = -1
                    
        return start_time

    # find end of sound
    def find_sound_end(self,rawAudio, start_time):
        window = int(0.125*44100)

        def index(seq, f):
            """Return the index of the first item in seq where f(item) == True."""
            return next((i for i in xrange(len(seq)) if f(seq[i])), None)

        counter = 0
        total = 0
    
        s = np.asarray(rawAudio)
        s = np.abs(s)
        e = len(s)
        s = self.rebin_time_fixed_width(s, window)
        start = int((start_time+0.1*44100)/window)
        s = s[start:]
        minimum = min(s)
        end = index(s, (lambda x: x < minimum * 2.0))
        
        if end is not None:
            end = (start + end) * window
            end = min(end + 0.1*44100, e)
            total += end
            counter += 1
            end_time = end
        else:
            end_time = -1
    
        return end_time

    # only for testing purposes
    def test(self, data_path):
        
        action = "push"
        obj = "pen"        

        input_file = open(os.path.join(data_path, action, obj + '.rawsound'), 'r')
        arr = array('d')
        arr.fromstring(input_file.read())
        sound = arr.tolist()
        
        input_file.close()

        # calculate FFT of raw sound
        sound_fft = self.calculate_fft(sound)

        # do classification, return object names and beliefs
        beliefs, names = self.classify(sound_fft)

        print names.keys()
        print names.values()

    # run the node forever
    def run(self):
        rospy.spin()

    def reset(self):
        pass

if __name__ == '__main__':

    node = classifyNode()
    #node.test('/tmp/sounds')
    node.run()

