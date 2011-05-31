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

import pickle
import math
import os
import time
from fnmatch import fnmatch
from array import array
from operator import itemgetter
import pickle

import roslib; roslib.load_manifest('ua_audio_capture')
import rospy

import pylab as pl
import numpy as np
import matplotlib
from scipy.ndimage.filters import gaussian_filter1d
from scikits.audiolab import Format, Sndfile

import nwalign as nw
from scikits.learn.cluster import SelfOrganizingMap
from Bio import kNN

from multiprocessing import Pool
from multiprocessing import cpu_count

from itertools import ifilter


def print_confusion_matrix(o, c):
    result = '    '
    
    num_obj = len(o)
    horiz_tot = c.sum(axis=1)
    vert_tot = c.sum(axis=0)
    
    for idx in range(num_obj):
        result = '%s %4s' % (result, idx)
        
    result += ' %4s  %4s\n' % ('Tot', 'Cor%')
    
    for idx1 in range(num_obj):
        result += '%4d' % idx1
        
        for idx2 in range(num_obj):
            result += (' %4d' % c[idx1][idx2])
            
        # make sure we don't divide by zero
        pc = 0 if horiz_tot[idx1] == 0 else int(float(c[idx1][idx1]) / horiz_tot[idx1] * 100)
        result += ' %4d  %4d%%\n' % (horiz_tot[idx1], pc)
        
    result += '%4s' % 'Tot'
    
    for idx in range(num_obj):
        # make sure we don't divide by zero
        result += ' %4d' % vert_tot[idx]
        
    result += '\n'
    result += '%4s' % 'Cor%'
    
    for idx in range(num_obj):
        # make sure we don't divide by zero
        pc = 0 if vert_tot[idx] == 0 else int(float(c[idx][idx]) / vert_tot[idx] * 100)
        result += ' %4s' % (str(pc) + '%')
        
    result += '\n'
    print result


def pretty_print_knn_probs(true_label, predicted_label, probs, n=5):
    sorted_probs = sorted(probs.iteritems(), key=itemgetter(1), reverse=True)[:n]
    
    print '%s --- [%s (true), %s (predicted)]' % (str(true_label == predicted_label), true_label, predicted_label)
    
    for label, prob in sorted_probs:
        print '%.4f --- %s' % (prob, label)
        
    print


def pretty_print_totals(object_count_by_actions):
    for act in object_count_by_actions:
        print act
        for obj in object_count_by_actions[act]:
            print '\t', obj, '---', object_count_by_actions[act][obj]
        print '\n'


def index_where(seq, f):
    """Return the index of the first item in seq where f(item) == True."""
    return next((i for i in xrange(len(seq)) if f(seq[i])), None)


def reverse_index_where(seq, f):
    """Return the index of the last item in seq where f(item) == True."""
    return next((i for i in xrange(len(seq) - 1, -1, -1) if f(seq[i])), None)


def filter_by_action(action_labels, object_labels, processed_ffts, needed):
    res_action_labels = []
    res_object_labels = []
    res_processed_ffts = []
    
    for idx,act in enumerate(action_labels):
        if act in needed:
            res_action_labels.append(act)
            res_object_labels.append(object_labels[idx])
            res_processed_ffts.append(processed_ffts[idx])
            
    return res_action_labels, res_object_labels, res_processed_ffts


def save_wav(sound, action_label, object_label):
    wav_path = '/tmp/new_wav'
    filename = os.path.join(wav_path, action_label + '-' + object_label + '-' + str(time.time()) + '.wav')
    format = Format('wav')

    print 'writing', filename, '...',

    f = Sndfile(filename, 'w', format, 1, 44100)
    f.write_frames(sound)
    f.close()
    print 'DONE'



class AudioClassifier():
    def __init__(self):
        self.params = {'som_size':             6,
                       'som_iterations':       10000,
                       'som_learning_rate':    0.05,
                       'nw_gap_open':          0,
                       'nw_gap_extend':        -5,
                       'knn_k':                10,
                       'fft_n':                512,
                       'fft_overlap':          256,
                       'fft_freq_bins':        17,
                       'rebin_window':         0.1,
                       'start_offset':         0.15,
                       'end_offset':           0.15,
                      }
                      
        self.data_paths = ['/tmp/robot_sounds/raw/drop',
                           '/tmp/robot_sounds/raw/mixed',
                           '/tmp/robot_sounds/raw/push',
                           '/tmp/robot_sounds/raw/shake_roll',
                           '/tmp/robot_sounds/raw/shake_pitch',
                          ]
                          
        self.action_names = ['grasp',        # 0
                             'lift',         # 1
                             'drop',         # 2
                             'shake_roll',   # 3
                             'place',        # 4
                             'push',         # 5
                             'shake_pitch',  # 6
                            ]
                            
        self.object_names = ['pink_glass',           # 0
                             'german_ball',          # 1
                             'blue_cup',             # 2
                             'blue_spiky_ball',      # 3
                             'screw_box',            # 4
                             'wire_spool',           # 5
                             'sqeaky_ball',          # 6
                             'duck_tape_roll',       # 7
                             'ace_terminals',        # 8
                             'chalkboard_eraser',    # 9
                            ]


    def stringify_sequence(self, seq):
        """
        input  = [(1,2),(2,3) ...]
        output = 'ab....'
        """
        map_side = self.params['som_size']
        result = []
        for t in seq:
            idx = t[0] * map_side + t[1]
            ascii_idx = idx + 48
            result.append(chr(ascii_idx))
        return ''.join(result)


    def generate_cost_matrix(self):
        map_side = self.params['som_size']
        result = '   '
        
        for i in range(map_side):
            for j in range(map_side):
                idx = i * map_side + j
                ascii_idx = idx + 48
                result += chr(ascii_idx)
                result += '  '
                
        result += '*\n'
        
        def make_row(x, y):
            row = ''
            ascii_idx = x * map_side + y + 48
            row += chr(ascii_idx)
            row += '  '
            
            for i in range(map_side):
                for j in range(map_side):
                    dist = math.sqrt(math.pow(j - y, 2.0) + math.pow(i - x, 2.0)) * 10     # euclidean
                    #dist = abs(j - y) + abs(i - x)  # manhattan
                    row += '%d' % -int(dist)
                    row += '  '
                    
            row += '-10\n'
            return row
            
        for i in range(map_side):
            for j in range(map_side):
                result += make_row(i, j)
                
        result += '*  '
        result += '-10  ' * (map_side * map_side)
        result += '1\n'
        
        outfile = open('/tmp/som.costs', 'w')
        outfile.write(result)
        outfile.close()
        
        return result


    def sound_seq_distance_str(self, seq1_str, seq2_str):
        gopen = self.params['nw_gap_open']
        gextend = self.params['nw_gap_extend']
        seq1_str = np.asanyarray(seq1_str)
        seq2_str = np.asanyarray(seq2_str)
        
        align = nw.global_align(seq1_str.tostring(), seq2_str.tostring(), gap_open=gopen, gap_extend=gextend, matrix='/tmp/som.costs')
        len1 = len(seq1_str.tostring())
        len2 = len(seq2_str.tostring())
        return (-nw.score_alignment(*align, gap_open=gopen, gap_extend=gextend, matrix='/tmp/som.costs'))/(len1+len2+0.0)


    def knn_weight_fn(self, x, y):
        dist = self.sound_seq_distance_str(x, y)
        return math.exp(-(dist-1)/0.5) / math.exp(0)


    def rebin_time_fixed_width(self, s, w):
        # filter first
        #s = gaussian_filter1d(s, 10, mode='constant')
        
        num_intervals = int(s.shape[0] / w + 1)
        result = []
        
        for i in range(num_intervals-1):
            start = i*w
            end = start + w
            m = s[start:end].mean()
            result.append(m)
            
        return np.asarray(result)


    def train_model(self, training_ffts, training_labels):
        """
        Takes a set of training examples + corresponding true labels and returns a
        trained SOM and kNN.
        """
        som_size = self.params['som_size']
        som_iterations = self.params['som_iterations']
        som_learning_rate = self.params['som_learning_rate']
        knn_k = self.params['knn_k']
        
        som = SelfOrganizingMap(size=som_size,
                                n_iterations=som_iterations,
                                learning_rate=som_learning_rate)
                                
        column_vectors = np.hstack(training_ffts).T
        np.random.shuffle(column_vectors)
        som.fit(column_vectors)
        
        training_sequences = []
        
        for example in training_ffts:
            seq = []
            
            for col in range(example.shape[1]):
                seq.append(som.bmu(example[:,col]))
                
            training_sequences.append(self.stringify_sequence(seq))
            
        knn_model = kNN.train(training_sequences, training_labels, knn_k)
        
        return som, knn_model


    def classify(self, sound_fft, som, knn_model):
        """
        Classify a new sound. Takes in a sound FFT, a SOM model and kNN model.
        Returns a tuple containing a highest probability label and a dictionary of
        all label probabilities.
        """
        sequence = []
        
        for col in range(sound_fft.shape[1]):
            sequence.append(som.bmu(sound_fft[:,col]))
            
        weights = kNN.calculate(knn_model,
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
                
        # make sure there are no 0's
        for k,v in weights.items():
            if v == 0: weights[k] = 1.0e-2
            
        # renormalize
        tot = sum(weights.values())
        for k,v in weights.items():
            weights[k] = v / float(tot)
            
        return most_class, weights


    def find_sound_start_new(self, data_paths, actions, objects):
        window = int(self.params['rebin_window'] * 44100)
        
        counter = {}
        total = {}
        
        for action_str in actions:
            counter[action_str] = 0.0
            total[action_str] = 0
            
        start_times_by_action = {}
        
        for action_str in actions:
            start_times_by_action[action_str] = {}
            for object_str in objects:
                start_times_by_action[action_str][object_str] = []
                
        for path in data_paths:
            print 'Processing sound descriptor files in %s' % path
            
            for fname in os.listdir(path):
                if fnmatch(fname, '*.desc'):
                    name = os.path.splitext(fname)[0]
                    desc_file = open(os.path.join(path, fname), 'r')
                    data_file = open(os.path.join(path, name + '.rawsound'), 'r')
                    
                    descriptors = desc_file.read().split('\n')
                    arr = array('d')
                    arr.fromstring(data_file.read())
                    audio_blob = arr.tolist()
                    
                    desc_file.close()
                    data_file.close()
                    
                    for descriptor in descriptors:
                        if not descriptor: continue
                        
                        action_id, object_id, offset, length = descriptor.split()
                        
                        # grab string labels for current action and object
                        action_str = actions[int(action_id)]
                        object_str = objects[int(object_id)]
                        
                        sound = np.asarray(audio_blob[int(offset):int(offset)+int(length)])
                        sound = np.abs(sound)
                        sound = self.rebin_time_fixed_width(sound, window)
                        maximum = max(sound)
                        
                        start = index_where(sound, (lambda x: x > 0.02))#maximum / 2.0))
                        
    #                    f = pl.figure()
    #                    ax = f.add_subplot(111)
    #                    im = pl.plot(sound)
    #                    pl.draw()
    #                    pl.show()
                        
                        
                        if start is not None:
                            start *= window
                            start = max(0, start - self.params['start_offset'] * 44100)
                            total[action_str] += start
                            counter[action_str] += 1
                            start_times_by_action[action_str][object_str].append(start)
                        else:
                            start_times_by_action[action_str][object_str].append(-1)
                            
        average_start_time = {}
        
        for action_str in actions:
            average_start_time[action_str] = int(total[action_str] / counter[action_str])
            
        for action_str in actions:
            for object_str in objects:
                for i in range(len(start_times_by_action[action_str][object_str])):
                    if start_times_by_action[action_str][object_str][i] == -1:
                        start_times_by_action[action_str][object_str][i] = average_start_time[action_str]
                        
        return start_times_by_action


    def find_sound_end_new(self, data_paths, actions, objects, start_times):
        window = int(self.params['rebin_window'] * 44100)
        
        counter = {}
        total = {}
        
        for action_str in actions:
            counter[action_str] = 0.0
            total[action_str] = 0
            
        end_times_by_action = {}
        
        for action_str in actions:
            end_times_by_action[action_str] = {}
            for object_str in objects:
                end_times_by_action[action_str][object_str] = []
                
        for path in data_paths:
            print 'Processing sound descriptor files in %s' % path
            
            for fname in os.listdir(path):
                if fnmatch(fname, '*.desc'):
                    name = os.path.splitext(fname)[0]
                    desc_file = open(os.path.join(path, fname), 'r')
                    data_file = open(os.path.join(path, name + '.rawsound'), 'r')
                    
                    descriptors = desc_file.read().split('\n')
                    arr = array('d')
                    arr.fromstring(data_file.read())
                    audio_blob = arr.tolist()
                    
                    desc_file.close()
                    data_file.close()
                    
                    for descriptor in descriptors:
                        if not descriptor: continue
                        
                        action_id, object_id, offset, length = descriptor.split()
                        
                        # grab string labels for current action and object
                        action_str = actions[int(action_id)]
                        object_str = objects[int(object_id)]
                        
                        sound = np.asarray(audio_blob[int(offset):int(offset)+int(length)])
                        sound = np.abs(sound)
                        last_index = len(sound)
                        sound = self.rebin_time_fixed_width(sound, window)
                        minimum = min(sound)
                        sample_idx = len(end_times_by_action[action_str][object_str])
                        start = int((start_times[action_str][object_str][sample_idx]+0.15*44100)/window)
                        sound = sound[start:]
                        maximum = max(sound)
                        end = reverse_index_where(sound, (lambda x: x > 0.02))#maximum / 2.0))
                        
                        if end is not None:
                            end = (start + end) * window
                            end = min(end + self.params['end_offset'] * 44100, last_index)
                            total[action_str] += end
                            counter[action_str] += 1
                            end_times_by_action[action_str][object_str].append(end)
                        else:
                            end_times_by_action[action_str][object_str].append(-1)
                            
                        sample_idx += 1
                        
        average_end_time = {}
        
        for action_str in actions:
            average_end_time[action_str] = int(total[action_str] / counter[action_str])
            
        for action_str in actions:
            for object_str in objects:
                for i in range(len(end_times_by_action[action_str][object_str])):
                    if end_times_by_action[action_str][object_str][i] == -1:
                        end_times_by_action[action_str][object_str][i] = average_end_time[action_str]
                        
        return end_times_by_action


    def calculate_fft_newdata(self, data_paths, actions, objects):
        """
        Given a path to the data, a list of actions and a list objects reads raw
        sound waves from files and computes FFTs. Returns a list of FFTs with a list
        of corresponding labels.
        """
        ####### Parameters ######
        sampling_rate = 44100
        fft_n = self.params['fft_n']
        fft_overlap = self.params['fft_overlap']
        fft_freq_bins = self.params['fft_freq_bins']
        #########################
        
        object_count_by_actions = {}
        for action_str in actions:
            object_count_by_actions[action_str] = {}
            for object_str in objects:
                object_count_by_actions[action_str][object_str] = 0
                
        processed_ffts = []
        object_labels = []
        action_labels = []
        
        #print 'Calculating start times...'
        start_times = self.find_sound_start_new(data_paths, actions, objects)
        #print start_times
        
        #print 'Calculating end times...'
        end_times = self.find_sound_end_new(data_paths, actions, objects, start_times)
        #print end_times
        
        sample_idx = 0
        
        for path in data_paths:
            print 'Processing sound descriptor files in %s' % path
            
            for fname in os.listdir(path):
                if fnmatch(fname, '*.desc'):
                    name = os.path.splitext(fname)[0]
                    desc_file = open(os.path.join(path, fname), 'r')
                    data_file = open(os.path.join(path, name + '.rawsound'), 'r')
                    
                    descriptors = desc_file.read().split('\n')
                    arr = array('d')
                    arr.fromstring(data_file.read())
                    audio_blob = arr.tolist()
                    
                    desc_file.close()
                    data_file.close()
                    
                    for descriptor in descriptors:
                        if not descriptor: continue
                        
                        action_id, object_id, offset, length = descriptor.split()
                        
                        # grab string labels for current action and object
                        action_str = actions[int(action_id)]
                        object_str = objects[int(object_id)]
                        
                        action_labels.append(action_str)
                        object_labels.append(object_str)
                        
                        sound = np.asarray(audio_blob[int(offset):int(offset)+int(length)])
                        idx = object_count_by_actions[action_str][object_str]
                        start = int(start_times[action_str][object_str][idx])
                        end = int(end_times[action_str][object_str][idx]) #start + int(fft_time_after_peak * sampling_rate)
                        sound = sound[start:end]
                        
    #                    f = pl.figure()
    #                    ax = f.add_subplot(111)
    #                    im = pl.plot(sound)
    #                    pl.draw()
    #                    pl.show()
                        
                        Pxx,freqs,t = matplotlib.mlab.specgram(sound, NFFT=fft_n, Fs=sampling_rate, noverlap=fft_overlap)
                        Pxx = 20 * np.log10(Pxx)
                        freqs /= 1000.0
                        
                        # bin resulting FFT into fft_freq_bins number of frequency bins
                        l = np.linspace(0, Pxx.shape[0], fft_freq_bins+1)
                        fft_binned = np.empty((fft_freq_bins,Pxx.shape[1]))
                        
                        for i in range(len(l)-1):
                            lis = int(l[i])
                            lie = int(l[i+1])
                            t = Pxx[lis:lie,:]
                            fft_binned[i] = t.mean(axis=0)
                            
                        processed_ffts.append(fft_binned)
                        print '[Sample %d] calculated FFT for action %s on object %s' % (sample_idx, action_str, object_str)
                        object_count_by_actions[action_str][object_str] += 1
                        sample_idx += 1
                        
    #                    f = pl.figure()
    #                    ax = f.add_subplot(111)
    #                    im = ax.pcolormesh(fft_binned)
    #                    pl.draw()
    #                    pl.show()
                        
        pretty_print_totals(object_count_by_actions)
        return action_labels, object_labels, processed_ffts


    def run_batch_training_classification_new(self, data_paths, action_names, object_names):
        action_labels, object_labels, processed_ffts = self.calculate_fft_newdata(data_paths, action_names, object_names)
        
        action_labels, object_labels, processed_ffts = filter_by_action(action_labels, object_labels, processed_ffts, ['push'])
        
        print 'len(processed_ffts)', len(processed_ffts)
        print 'len(action_labels)', len(action_labels)
        print 'len(object_labels)', len(object_labels)
        
        labels = np.asarray(object_labels)
        act_labels = np.asarray(action_labels)
        
        num_samplings = 5
        corr_arr = np.zeros(num_samplings)
        incorr_arr = np.zeros(num_samplings)
        confusion_matrix = np.zeros((len(object_names), len(object_names)), dtype=int)
        
        for sampling in range(num_samplings):
            print 'Sampling', sampling
            inds = range(len(processed_ffts))
            np.random.shuffle(inds)
            
            num_tot = len(processed_ffts)
            num_train = int(0.8 * num_tot)
            num_test = num_tot - num_train
            print 'Number of training instances', num_train
            print 'Number of testing instances', num_test
            
            train_set = [processed_ffts[idx] for idx in inds[:num_train]]
            test_set = [processed_ffts[idx] for idx in inds[num_train:]]
            
            train_labels = labels[inds][:num_train]
            test_labels = labels[inds][num_train:]
    #        print train_labels
    #        print '*'*100
    #        print test_labels
            
            act_train_labels = act_labels[inds][:num_train]
            act_test_labels = act_labels[inds][num_train:]
            print act_train_labels
            print '*'*100
            print act_test_labels
            
            l = {}
            for act in action_names:
                l[act] = []
                
            for idx,act in enumerate(act_train_labels):
                l[act].append(idx)
                
            soms = {}
            knns = {}
            for act in action_names:
                ts = [train_set[i] for i in l[act]]
                tl = [train_labels[i] for i in l[act]]
                
                if ts and tl:
                    som, knn_model = self.train_model(ts, tl)
                    soms[act] = som
                    knns[act] = knn_model
                
            # save som and knn_model off for later use
    #        pickle.dump(knn_model, open('/tmp/knn_model.pkl','w'))
    #        pickle.dump(som, open('/tmp/som.pkl','w'))
            
            correct = 0
            for idx, seq in enumerate(test_set):
                act = act_test_labels[idx]
                label, probs = self.classify(seq, soms[act], knns[act])
                pretty_print_knn_probs(test_labels[idx], label, probs)
                true_id = object_names.index(test_labels[idx])
                pred_id = object_names.index(label)
                confusion_matrix[true_id][pred_id] += 1
                if label == test_labels[idx]: correct += 1
            pct_correct = float(correct) / len(test_set)
            print 'Result: %.2f%% correct, %.2f%% incorrect' % (pct_correct*100, (1.0-pct_correct)*100)
            
            corr_arr[sampling] = pct_correct
            incorr_arr[sampling] = 1.0-pct_correct
            corr_sub = corr_arr[:sampling+1]
            incorr_sub = incorr_arr[:sampling+1]
            print 'Current correct mean/std/var   %.2f %.2f %.3f' % (corr_sub.mean(), corr_sub.std(), corr_sub.var())
            print 'Current incorrect mean/std/var %.2f %.2f %.3f\n' % (incorr_sub.mean(), incorr_sub.std(), incorr_sub.var())
            
        print '  Correct mean/std/var/max/min', corr_arr.mean(), corr_arr.std(), corr_arr.var(), corr_arr.max(), corr_arr.min()
        print 'Incorrect mean/std/var/max/min', incorr_arr.mean(), incorr_arr.std(), incorr_arr.var(), incorr_arr.max(), incorr_arr.min()
        
        print_confusion_matrix(object_names, confusion_matrix)
        
        for idx, obj in enumerate(object_names):
            print '%4d --- %s' % (idx, obj)
            
        return corr_arr.mean()


    def generate_fake_pdfs_by_action(self, data_paths, action_names, object_names, load_pickle=True, dump_pickle=False):
        action_labels, object_labels, processed_ffts = None, None, None
        
        if not load_pickle:
            action_labels, object_labels, processed_ffts = self.calculate_fft_newdata(data_paths, action_names, object_names)
        else:
            print 'Reading FFT data from pickle'
            input_pkl = open('/tmp/robot_sounds/fft/all_ffts.pkl', 'rb')
            action_labels, object_labels, processed_ffts = pickle.load(input_pkl)
            input_pkl.close()
            
        if dump_pickle:
            print 'Saving FFT data to pickle file'
            output = open('/tmp/robot_sounds/fft/all_ffts.pkl', 'wb')
            pickle.dump([action_labels,object_labels,processed_ffts], output)
            output.close()
            
        # do leave-one-out cross validation
        # also change the neighborhood a bit each time by randomly removing
        # some other objects
        
        act_obj_pdfs = {}
        
        # find action indicies first
        # will have a map of action -> object inidices, e.g. drop -> [1,4,6,9]
        act_inds = {}
        for act in action_names:
            act_inds[act] = [idx for idx,label in enumerate(action_labels) if label == act]
            
        n = 0.2
        
        # go through each action
        # grab an object
        for act in act_inds:
            # find object indicies for this action
            # will have a map of object -> inidices, e.g. german_ball -> [1,4,6,9]
            obj_inds = {}
            num_act_samples = 0
            for obj in object_names:
                obj_inds[obj] = [idx for idx,label in enumerate(object_labels) if idx in act_inds[act] and label == obj]
                print 'There are %d samples for object %s and action %s' % (len(obj_inds[obj]), obj, act)
                num_act_samples += len(obj_inds[obj])
                
            print 'There are %d samples for action %s' % (num_act_samples, act)
            
            obj_inds_perm = {}
            for obj in object_names:
                obj_inds_perm[obj] = np.random.permutation(obj_inds[obj]).tolist()[:20]
                
            # initialze everything to 0.0 for averaging
#            obj_pdfs = {}
#            for obj in object_names:
#                obj_pdfs[obj] = {}
#                for other_obj in object_names:
#                    obj_pdfs[obj][other_obj] = 0.0  # 1.0/len(object_names)
                    
            obj_pdfs = {}
            for obj in object_names:
                obj_pdfs[obj] = [] # will contain n different pdf for this object
                
            obj_totals = {}
            for obj in object_names:
                obj_totals[obj] = 0
                
            for obj in obj_inds:
                print 'processing (%s, %s) pair' % (act, obj)
                print 'we have %d samples for this object' % len(obj_inds[obj])
                
                rest_inds = []
                for l in obj_inds:
                    #print l, '!=', obj, 'adding', len(obj_inds[l]), 'samples'
                    #print sorted(obj_inds[l])
                    rest_inds.extend(obj_inds[l])
                        
                print 'Selected %s samples for leave-one-out validation' % str(obj_inds_perm[obj])
                
                # skip object with index ind
                for ind in obj_inds_perm[obj]:
                    obj_totals[obj] += 1
                    
                    # remove n random samples from other objects
                    num_remove = int(n*len(rest_inds))
                    to_remove = np.random.permutation(rest_inds)[:num_remove]
                    to_remove = np.append(to_remove, ind)
                    print 'Will remove %d samples (num_remove=%d)' % (len(to_remove), num_remove)
                    #print 'rest_inds'
                    #print sorted(rest_inds)
                    #print '\n\nto_remove'
                    #print sorted(to_remove)
                    
                    # this contains indices of all objects except the one we are leaving out and
                    # random sample that we removed in order to perturb neighborhoods a bit
                    train_inds = [idx for idx in rest_inds if idx not in to_remove]
                    #print '\n\ntrain_inds'
                    #print sorted(train_inds)
                    
                    train_action_labels = [action_labels[idx] for idx in train_inds]
                    train_object_labels = [object_labels[idx] for idx in train_inds]
                    train_processed_fft = [processed_ffts[idx] for idx in train_inds]
                    
                    # training
                    print 'Training model for object %s (nsamples=%d)...' % (obj, len(train_processed_fft))
                    som, knn_model = self.train_model(train_processed_fft, train_object_labels)
                    
                    # validation
                    print 'Validating model...'
                    label, probs = self.classify(processed_ffts[ind], som, knn_model)
                    
                    obj_pdfs[obj].append(probs)
                    
                    #for other_obj in probs:
                    #    obj_pdfs[obj][other_obj] += probs[other_obj]
                        
                    print 'Action', action_labels[ind]
                    pretty_print_knn_probs(object_labels[ind], label, probs, 7)
                    #true_id = object_names.index(object_labels[ind])
                    #pred_id = object_names.index(label)
                    #confusion_matrix[true_id][pred_id] += 1
                    #if label == test_labels[idx]: correct += 1
                    
                # renormalize
                #for other_obj in obj_pdfs[obj]:
                #    if obj_totals[obj] != 0:
                #        obj_pdfs[obj][other_obj] /= obj_totals[obj]
                        
                # print pdf for current object
                print 'Finished calculating PDFs for object %s' % obj
#                sorted_probs = sorted(obj_pdfs[obj].iteritems(), key=itemgetter(1), reverse=True)
#                for other_obj in sorted_probs:
#                    print '\t', other_obj
                    
            print 'ACTION %s DONE' % act
            print obj_pdfs
            act_obj_pdfs[act] = obj_pdfs
            
        print 'ALL DONE'
        print act_obj_pdfs
        out = open('/tmp/obj_pdf.pkl', 'wb')
        pickle.dump(act_obj_pdfs, out)
        out.close()


    def generate_confusion_matrix(self, data):
        action_name = data[0]
        action_labels = data[1]
        object_labels = data[2]
        action_ffts = data[3]
        sampling = data[4]
        
        num_categories = len(self.object_names)
        num_ffts = len(action_ffts)
        
        labels = np.asarray(object_labels)
        
        inds = range(num_ffts)
        np.random.shuffle(inds)
        
        num_tot = num_ffts
        num_train = int(0.8 * num_tot)
        num_test = num_tot - num_train
        
        train_set = [action_ffts[idx] for idx in inds[:num_train]]
        test_set = [action_ffts[idx] for idx in inds[num_train:]]
        
        train_labels = labels[inds][:num_train]
        test_labels = labels[inds][num_train:]
        
        print '[%s, %d] Training model (train = %d, test = %d)...' % (action_name.upper(), sampling, num_train, num_test)
        som, knn_model = self.train_model(train_set, train_labels)
        
        del train_set
        
        sampling_probs = np.zeros((num_test,num_categories), dtype=float)
        object_names = []
        
        for idx, seq in enumerate(test_set):
            if idx % 25 == 0:
                print 'Action %s, Sampling %d, Test instance %d/%d' % (action_name.upper(), sampling, idx, num_test)
                
            label, probs = self.classify(seq, som, knn_model)
            object_names.append(test_labels[idx])
            
            # copy over the probabilities in correct order
            for cat_id,obj in enumerate(self.object_names):
                sampling_probs[idx,cat_id] = probs[obj]
                
        return sampling, action_name, object_names, sampling_probs


    def run(self):
        self.generate_cost_matrix()
        res = 0.0
        try:
            res = self.run_batch_training_classification_new(self.data_paths, self.action_names, self.object_names)
        except Exception as e:
            print e
            res = 0.0
        return res


    def run_pdfs(self):
        self.generate_cost_matrix()
        self.generate_fake_pdfs_by_action(self.data_paths, self.action_names, self.object_names)#, False, True)


    def run_confusion(self):
        self.generate_cost_matrix()
        self.generate_confusion_matrices_by_action()


    def generate_confusion_matrices_by_action(self):
        print 'Reading FFT data from pickle...'
        
        input_pkl = open('/tmp/robot_sounds/fft/all_ffts.pkl', 'rb')
        action_labels_all, object_labels_all, processed_ffts_all = pickle.load(input_pkl)
        input_pkl.close()
        
        print 'done'
        
        num_cpus = cpu_count()
        print 'Using %d CPUs for experiments' % num_cpus
        
        num_samplings = 15
        num_objects = len(self.object_names)
        
        proc_data = []
        for action_name in self.action_names:
            action_labels, object_labels, processed_ffts = filter_by_action(action_labels_all, object_labels_all, processed_ffts_all, [action_name])
            
            for sampling in range(num_samplings):
                proc_data.append((self, [action_name, action_labels, object_labels, processed_ffts, sampling]))
                
        del action_labels_all
        del object_labels_all
        del processed_ffts_all
        
        pool = Pool(processes=2)
        res = pool.map(C_m2, proc_data)
        #res = C_m2(proc_data[0])
        
        probs_by_action = {}
        
        for action_name in self.action_names:
            probs_by_action[action_name] = {}
            
            for object_name in self.object_names:
                probs_by_action[action_name][object_name] = np.zeros((0,num_objects))
                
        for sampling,action_name,object_names,sampling_probs in res:
            for idx,object_name in enumerate(object_names):
                probs_by_action[action_name][object_name] = np.vstack((probs_by_action[action_name][object_name],sampling_probs[idx]))
            
        for action_name in self.action_names:
            for object_name in self.object_names:
                print '[%s, %s] %s' % (action_name.upper(), object_name, str(probs_by_action[action_name][object_name].shape))
                
        output = open('/tmp/proportions.pkl', 'wb')
        pickle.dump([probs_by_action,self.object_names,self.action_names], output)
        output.close()


def C_m2( ar, **kwar ):
    return AudioClassifier.generate_confusion_matrix( *ar, **kwar )


if __name__ == '__main__':
    ac = AudioClassifier()
#    ac.run(); exit(1)
#    ac.run_pdfs(); exit(1)
    ac.run_confusion(); exit(1)

