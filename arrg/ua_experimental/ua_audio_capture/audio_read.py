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


def get_fake_pdfs():
    pdfs = {'sqeaky_ball':          {'sqeaky_ball': 0.2982929809968054, 'blue_spiky_ball': 0.099999902266118892, 'ace_terminals': 0.060357655927000639, 'chalkboard_eraser': 0.040555575179976322, 'blue_cup': 0.080197959207489511, 'screw_box': 0.00099295092550315159, 'german_ball': 0.25833325225323234, 'duck_tape_roll': 0.060336887307212914, 'wire_spool': 0.080158572883921475, 'pink_glass': 0.020774263052739737},
            'blue_spiky_ball':      {'sqeaky_ball': 0.020774954043208796, 'blue_spiky_ball': 0.33775602281376549, 'ace_terminals': 0.060397595552642777, 'chalkboard_eraser': 0.060417278450281796, 'blue_cup': 0.060417278450281817, 'screw_box': 0.060376944092960037, 'german_ball': 0.060357300287471237, 'duck_tape_roll': 0.060318012560147925, 'wire_spool': 0.15950294281759339, 'pink_glass': 0.11968167093164692},
            'ace_terminals':        {'sqeaky_ball': 0.020774164731283639, 'blue_spiky_ball': 0.020754520925794832, 'ace_terminals': 0.43678677490814, 'chalkboard_eraser': 0.020754520925794821, 'blue_cup': 0.11982050239546738, 'screw_box': 0.13954282336945165, 'german_ball': 0.040535833053031406, 'duck_tape_roll': 0.08031690827005103, 'wire_spool': 0.060396806240717603, 'pink_glass': 0.06031714518026797},
            'chalkboard_eraser':    {'sqeaky_ball': 0.020755014098197168, 'blue_spiky_ball': 0.13966226532962694, 'ace_terminals': 0.080098950479906908, 'chalkboard_eraser': 0.21886924199349514, 'blue_cup': 0.060337321250309384, 'screw_box': 0.020774657903685968, 'german_ball': 0.080118633377545947, 'duck_tape_roll': 0.06035696505579817, 'wire_spool': 0.040516721395749467, 'pink_glass': 0.27851022911568507},
            'blue_cup':             {'sqeaky_ball': 0.020774262856881422, 'blue_spiky_ball': 0.11990026158151475, 'ace_terminals': 0.15948351676738076, 'chalkboard_eraser': 0.040555574984118006, 'blue_cup': 0.31803405912568977, 'screw_box': 0.11964266069638566, 'german_ball': 0.11976353877389016, 'duck_tape_roll': 0.060317243305865763, 'wire_spool': 0.020774262856881422, 'pink_glass': 0.020754619051392607}, 
            'screw_box':            {'sqeaky_ball': 0.060396627578176341, 'blue_spiky_ball': 0.060396627578176335, 'ace_terminals': 0.15940454555908562, 'chalkboard_eraser': 0.020774954630786057, 'blue_cup': 0.099920951310742787, 'screw_box': 0.29823380950304068, 'german_ball': 0.060337617977409455, 'duck_tape_roll': 0.060357300875048495, 'wire_spool': 0.060357300875048495, 'pink_glass': 0.11982026411248592},
            'german_ball':          {'sqeaky_ball': 0.040576383363728187, 'blue_spiky_ball': 0.099920358837588166, 'ace_terminals': 0.020794045055270445, 'chalkboard_eraser': 0.060297737776931495, 'blue_cup': 0.00099305003039482781, 'screw_box': 0.00099305003039482781, 'german_ball': 0.47638797924919052, 'duck_tape_roll': 0.04055567428486799, 'wire_spool': 0.17936338374014235, 'pink_glass': 0.080118337631491371},
            'duck_tape_roll':       {'sqeaky_ball': 0.060397200112952278, 'blue_spiky_ball': 0.099979566885504662, 'ace_terminals': 0.10004043161939136, 'chalkboard_eraser': 0.13942430754606364, 'blue_cup': 0.040516583119777273, 'screw_box': 0.08015886843470002, 'german_ball': 0.060436586436520313, 'duck_tape_roll': 0.23884810198251555, 'wire_spool': 0.060397200112952264, 'pink_glass': 0.11980115374962282},
            'wire_spool':           {'sqeaky_ball': 0.00099295131663744603, 'blue_spiky_ball': 0.00099295131663744603, 'ace_terminals': 0.040555575571110621, 'chalkboard_eraser': 0.099959529207781159, 'blue_cup': 0.020774263443874032, 'screw_box': 0.060436291276876032, 'german_ball': 0.10005893278630999, 'duck_tape_roll': 0.13958226942980551, 'wire_spool': 0.41686594271901073, 'pink_glass': 0.11978129293195712},
            'pink_glass':           {'sqeaky_ball': 0.00099364113136965232, 'blue_spiky_ball': 0.00099364113136965232, 'ace_terminals': 0.11984097382042898, 'chalkboard_eraser': 0.080179005607867157, 'blue_cup': 0.020774953258606243, 'screw_box': 0.080118928732466205, 'german_ball': 0.17904621901263229, 'duck_tape_roll': 0.080118928732466205, 'wire_spool': 0.15954323729816233, 'pink_glass': 0.27839047127463129}
           }
           
    # shake_pitch
    {'sqeaky_ball':         {'sqeaky_ball': 0.0, 'blue_spiky_ball': 0.0, 'ace_terminals': 0.0, 'chalkboard_eraser': 0.0, 'blue_cup': 0.0, 'screw_box': 0.0, 'german_ball': 0.0, 'duck_tape_roll': 0.0, 'wire_spool': 0.0, 'pink_glass': 0.0},
     'blue_spiky_ball':     {'sqeaky_ball': 0.00099157208005896578, 'blue_spiky_ball': 0.89232632415978408, 'ace_terminals': 0.00099157208005896578, 'chalkboard_eraser': 0.00099157208005896578, 'blue_cup': 0.040495303893870245, 'screw_box': 0.00099157208005896578, 'german_ball': 0.00099157208005896578, 'duck_tape_roll': 0.00099157208005896578, 'wire_spool': 0.00099157208005896578, 'pink_glass': 0.060237367385933729},
     'ace_terminals':       {'sqeaky_ball': 0.00099137524187078259, 'blue_spiky_ball': 0.00099137524187078259, 'ace_terminals': 0.81280381329935403, 'chalkboard_eraser': 0.00099137524187078259, 'blue_cup': 0.00099137524187078259, 'screw_box': 0.17926518476568037, 'german_ball': 0.00099137524187078259, 'duck_tape_roll': 0.00099137524187078259, 'wire_spool': 0.00099137524187078259, 'pink_glass': 0.00099137524187078259},
     'chalkboard_eraser':   {'sqeaky_ball': 0.0, 'blue_spiky_ball': 0.0, 'ace_terminals': 0.0, 'chalkboard_eraser': 0.0, 'blue_cup': 0.0, 'screw_box': 0.0, 'german_ball': 0.0, 'duck_tape_roll': 0.0, 'wire_spool': 0.0, 'pink_glass': 0.0},
     'blue_cup':            {'sqeaky_ball': 0.0406150007469697, 'blue_spiky_ball': 0.099900005712212969, 'ace_terminals': 0.00099235923753571782, 'chalkboard_eraser': 0.00099235923753571782, 'blue_cup': 0.51597251601493466, 'screw_box': 0.00099235923753571782, 'german_ball': 0.0406150007469697, 'duck_tape_roll': 0.00099235923753571782, 'wire_spool': 0.00099235923753571782, 'pink_glass': 0.29793568059123443},
     'screw_box':           {'sqeaky_ball': 0.00099147356332688393, 'blue_spiky_ball': 0.00099147356332688393, 'ace_terminals': 0.13948353705539043, 'chalkboard_eraser': 0.00099147356332688393, 'blue_cup': 0.00099147356332688393, 'screw_box': 0.85258467443799513, 'german_ball': 0.00099147356332688393, 'duck_tape_roll': 0.00099147356332688393, 'wire_spool': 0.00099147356332688393, 'pink_glass': 0.00099147356332688393},
     'german_ball':         {'sqeaky_ball': 0.00099167020623908641, 'blue_spiky_ball': 0.13958294004750899, 'ace_terminals': 0.00099167020623908641, 'chalkboard_eraser': 0.00099167020623908641, 'blue_cup': 0.00099167020623908641, 'screw_box': 0.00099167020623908641, 'german_ball': 0.71389242846130885, 'duck_tape_roll': 0.00099167020623908641, 'wire_spool': 0.00099167020623908641, 'pink_glass': 0.13958294004750896},
     'duck_tape_roll':      {'sqeaky_ball': 0.0, 'blue_spiky_ball': 0.0, 'ace_terminals': 0.0, 'chalkboard_eraser': 0.0, 'blue_cup': 0.0, 'screw_box': 0.0, 'german_ball': 0.0, 'duck_tape_roll': 0.0, 'wire_spool': 0.0, 'pink_glass': 0.0},
     'wire_spool':          {'sqeaky_ball': 0.0, 'blue_spiky_ball': 0.0, 'ace_terminals': 0.0, 'chalkboard_eraser': 0.0, 'blue_cup': 0.0, 'screw_box': 0.0, 'german_ball': 0.0, 'duck_tape_roll': 0.0, 'wire_spool': 0.0, 'pink_glass': 0.0},
     'pink_glass':          {'sqeaky_ball': 0.00099216220407155413, 'blue_spiky_ball': 0.15928520219398343, 'ace_terminals': 0.00099216220407155413, 'chalkboard_eraser': 0.00099216220407155413, 'blue_cup': 0.21869010202217026, 'screw_box': 0.00099216220407155413, 'german_ball': 0.00099216220407155413, 'duck_tape_roll': 0.00099216220407155413, 'wire_spool': 0.00099216220407155413, 'pink_glass': 0.6150795603553455}}



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




import time
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
                       'knn_k':                5,
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
            if v == 0: weights[k] = 0.001
            
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
                    pretty_print_knn_probs(object_labels[ind], label, probs)
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

    def eval_fn(self, chromosome):
        for i,param in enumerate(sorted(self.params.keys())):
            print 'setting param %s to %f' % (param, chromosome[i])
            self.params[param] = chromosome[i]
            
        res = 0.0
        
        if self.params['fft_n'] > self.params['fft_overlap']:
            res = self.run()
            
        print 'Evaluated to', res
        return res

#from pyevolve import G1DList
#from pyevolve import GSimpleGA
#from pyevolve import Selectors
#from pyevolve import Mutators
#from pyevolve import Initializators
#from pyevolve import GAllele


if __name__ == '__main__':
    ac = AudioClassifier()
#    ac.run(); exit(1)
    ac.run_pdfs(); exit(1)
    
#    limits = {'som_size':           [2,8,False],
#              'som_iterations':     [1000,10000,False],
#              'som_learning_rate':  [0.01, 0.1,True],
#              'nw_gap_open':        [-20,0,False],
#              'nw_gap_extend':      [-20,0,False],
#              'knn_k':              [1,11,False],
#              'fft_n':              [32,1024,False],
#              'fft_overlap':        [32,1024,False],
#              'fft_freq_bins':      [5,129,False],
#              'rebin_window':       [0.01,0.5,True],
#              'start_offset':       [0.01,0.5,True],
#              'end_offset':         [0.01,0.5,True],
#             }
#             
#    setOfAlleles = GAllele.GAlleles()
#    
#    for i in sorted(limits.keys()):
#        a = GAllele.GAlleleRange(limits[i][0], limits[i][1], limits[i][2])
#        setOfAlleles.add(a)
#        
#    genome = G1DList.G1DList(len(limits))
#    genome.setParams(allele=setOfAlleles)
#    
#    genome.evaluator.set(ac.eval_fn)
#    genome.mutator.set(Mutators.G1DListMutatorAllele)
#    genome.initializator.set(Initializators.G1DListInitializatorAllele)
#    
#    ga = GSimpleGA.GSimpleGA(genome)
#    ga.selector.set(Selectors.GRouletteWheel)
#    ga.setGenerations(500)
#    
#    ga.evolve(freq_stats=1)
#    
#    # Best individual
#    print ga.bestIndividual()
    
