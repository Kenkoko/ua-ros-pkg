#!/usr/bin/env python

import roslib; roslib.load_manifest('ua_audio_capture')

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


def stringify_sequence(seq):
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

def matrix():
    map_side = 6
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
    
    print result
    return result

def sound_seq_distance_str(seq1_str, seq2_str):
    seq1_str = np.asanyarray(seq1_str)
    seq2_str = np.asanyarray(seq2_str)
    
    align = nw.global_align(seq1_str.tostring(), seq2_str.tostring(), gap_open=0, gap_extend=-5, matrix='/tmp/som.costs')
    len1 = len(seq1_str.tostring())
    len2 = len(seq2_str.tostring())
    return (-nw.score_alignment(*align, gap_open=0, gap_extend=-5, matrix='/tmp/som.costs'))/(len1+len2+0.0)

def knn_weight_fn(x, y):
    #dist = sound_seq_distance_str(x, y)
    return 1#math.exp(dist)

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

def rebin_time_fixed_width(s, w):
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

def find_sound_start(data_path, action, objects):
    window = int(0.125*44100)

    def index(seq, f):
        """Return the index of the first item in seq where f(item) == True."""
        return next((i for i in xrange(len(seq)) if f(seq[i])), None)

    counter = 0
    total = 0
    
    start_times = {}

    for fidx, obj in enumerate(objects):
        desc_file = open(os.path.join(data_path, action, obj + '.desc'), 'r')
        input_file = open(os.path.join(data_path, action, obj + '.rawsound'), 'r')
        
        desc = desc_file.read().split('\n')
        arr = array('d')
        arr.fromstring(input_file.read())
        sound = arr.tolist()
        
        desc_file.close()
        input_file.close()
        
        start_times[obj] = []

        for idx, d in enumerate(desc):
            if d:
                _, _, offset, length = d.split()
                s = np.asarray(sound[int(offset):int(offset)+int(length)])
                s = np.abs(s)
                s = rebin_time_fixed_width(s, window)
                maximum = max(s)
#                if idx == 4:
#                    pl.plot(s)
#                    pl.draw()
#                    pl.show()
#                continue
                start = index(s, (lambda x: x > maximum / 2.0))
                
                if start is not None:
                    start *= window
                    start = max(0, start - 0.1*44100)
                    total += start
                    counter += 1
                    start_times[obj].append(start)
                else:
                    start_times[obj].append(-1)
#    exit(1)
    average_start_time = int(total / counter)
    
    for obj in start_times.keys():
        for i in range(len(start_times[obj])):
            if start_times[obj][i] == -1: start_times[obj][i] = average_start_time
            
    return start_times


def find_sound_end(data_path, action, objects, start_times):
    window = int(0.125*44100)

    def index(seq, f):
        """Return the index of the first item in seq where f(item) == True."""
        return next((i for i in xrange(len(seq)) if f(seq[i])), None)

    counter = 0
    total = 0
    
    end_times = {}

    for fidx, obj in enumerate(objects):
        desc_file = open(os.path.join(data_path, action, obj + '.desc'), 'r')
        input_file = open(os.path.join(data_path, action, obj + '.rawsound'), 'r')
        
        desc = desc_file.read().split('\n')
        arr = array('d')
        arr.fromstring(input_file.read())
        sound = arr.tolist()
        
        desc_file.close()
        input_file.close()
        
        end_times[obj] = []

        for idx, d in enumerate(desc):
            if d:
                _, _, offset, length = d.split()
                s = np.asarray(sound[int(offset):int(offset)+int(length)])
                s = np.abs(s)
                e = len(s)
                s = rebin_time_fixed_width(s, window)
                start = int((start_times[obj][idx]+0.1*44100)/window)
                s = s[start:]
                minimum = min(s)
                end = index(s, (lambda x: x < minimum * 2.0))
                
                if end is not None:
                    end = (start + end) * window
                    end = min(end + 0.1*44100, e)
                    total += end
                    counter += 1
                    end_times[obj].append(end)
                else:
                    end_times[obj].append(-1)
#    exit(1)
    average_end_time = int(total / counter)
    
    for obj in end_times.keys():
        for i in range(len(end_times[obj])):
            if end_times[obj][i] == -1: end_times[obj][i] = average_end_time
            print obj, i, start_times[obj][i], end_times[obj][i]
            
    return end_times


def train_model(training_ffts, training_labels):
    """
    Takes a set of training examples + corresponding true labels and returns a
    trained SOM and kNN.
    """
    ### Model parameters ###
    som_size = 6
    som_iterations = 10000
    som_learning_rate = 0.05
    knn_k = 5
    ########################
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
            
        training_sequences.append(stringify_sequence(seq))
        
    knn_model = kNN.train(training_sequences, training_labels, knn_k)
    
    return som, knn_model


def classify(sound_fft, som, knn_model):
    """
    Classify a new sound. Takes in a sound FFT, a SOM model and kNN model.
    Returns a tuple containing a highest probability label and a dictionary of
    all label probabilities.
    """
    sequence = []
    
    for col in range(sound_fft.shape[1]):
        sequence.append(som.bmu(sound_fft[:,col]))
        
    weights = kNN.calculate(knn_model,
                            stringify_sequence(sequence),
                            weight_fn=knn_weight_fn,
                            distance_fn=sound_seq_distance_str)
                          
    sum_weights = float(sum(weights.values()))
    
    most_class = None
    most_weight = None
    
    for klass, weight in weights.items():
        weights[klass] = weight / sum_weights
        
        if most_class is None or weight > most_weight:
            most_class = klass
            most_weight = weight
            
    return most_class, weights


def pretty_print_knn_probs(true_label, predicted_label, probs, n=5):
    sorted_probs = sorted(probs.iteritems(), key=itemgetter(1), reverse=True)[:n]
    
    print '%s --- [%s (true), %s (predicted)]' % (str(true_label == predicted_label), true_label, predicted_label)
    
    for label, prob in sorted_probs:
        print '%.4f --- %s' % (prob, label)
        
    print

def calculate_fft(data_path, action, objects):
    """
    Given a path to the data, a list of actions and a list objects reads raw
    sound waves from files and computes FFTs. Returns a list of FFTs with a list
    of corresponding labels.
    """
    ####### Parameters ######
    sampling_rate = 44100
    fft_n = 512
    fft_overlap = 256
    fft_time_after_peak = 2.0
    fft_freq_bins = 33
    #########################
    
    processed_ffts = []
    object_labels = []
    
    print 'Calculating start times...'
    start_times = find_sound_start(data_path, action, objects)
    
    print 'Calculating end times...'
    end_times = find_sound_end(data_path, action, objects, start_times)
    
    for fidx, obj in enumerate(objects):
        desc_file = open(os.path.join(data_path, action, obj + '.desc'), 'r')
        input_file = open(os.path.join(data_path, action, obj + '.rawsound'), 'r')
        
        desc = desc_file.read().split('\n')
        arr = array('d')
        arr.fromstring(input_file.read())
        sound = arr.tolist()
        
        desc_file.close()
        input_file.close()

        for idx, d in enumerate(desc):
            if d:
                object_labels.append(obj)
                _, _, offset, length = d.split()
                s = np.asarray(sound[int(offset):int(offset)+int(length)])
                start = int(start_times[obj][idx])
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
                    
                processed_ffts.append(fft_binned)
                print 'Calculated FFT for sample %d of object %s' % (idx, obj)
                
    #object_labels = np.asarray(object_labels)
    
    return processed_ffts, object_labels

from scikits.learn.cluster import AffinityPropagation

def test_affinity(data_path, actions, objects):
    processed_ffts, labels = calculate_fft(data_path, actions[0], objects)
    act_lab = [actions[0]]*len(labels)
    
    p, l = calculate_fft(data_path, actions[1], objects)
    processed_ffts.extend(p)
    labels.extend(l)
    act_lab.extend([actions[1]]*len(l))
    
    p, l = calculate_fft(data_path, actions[2], objects)
    processed_ffts.extend(p)
    labels.extend(l)
    act_lab.extend([actions[2]]*len(l))
    
    p, l = calculate_fft(data_path, actions[3], objects)
    processed_ffts.extend(p)
    labels.extend(l)
    act_lab.extend([actions[3]]*len(l))

    #labels = act_lab
    #objects = actions
    print labels
    print 'len(processed_ffts)', len(processed_ffts)
    print 'len(labels)', len(labels)
    
    labels = np.asarray(labels)
    
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
    print train_labels
    print '*'*100
    print test_labels
    
    som, knn_model = train_model(train_set, train_labels)
    
    S = np.zeros((num_train, num_train))
    
    for idx1 in range(len(train_set)):
        print 'processing example', idx1
        fft1 = train_set[idx1]
        sequence1 = []
        
        for col in range(fft1.shape[1]):
            sequence1.append(som.bmu(fft1[:,col]))
            
        for idx2 in range(idx1+1, len(train_set)):
            fft2 = train_set[idx2]
            sequence2 = []
            
            for col in range(fft2.shape[1]):
                sequence2.append(som.bmu(fft2[:,col]))
                
            S[idx1,idx2] = -sound_seq_distance_str(stringify_sequence(sequence1), stringify_sequence(sequence2))
            S[idx2,idx1] = S[idx1,idx2]
            
    print S
    print S.shape
    
    af = AffinityPropagation()
    af.fit(S)
    print 'estimated number of clusters', len(af.cluster_centers_indices_)
    print 'cluster indices are', af.cluster_centers_indices_
    for idx in af.cluster_centers_indices_:
        print train_labels[idx]
    print 'labels'
    print af.labels_
    
    for i in range(len(af.cluster_centers_indices_)):
        inds = np.argwhere(af.labels_ == i).flatten()
        print 'center label', train_labels[af.cluster_centers_indices_[i]], 'af index', i
        print labels[inds]


def run_batch_training_classification(data_path, actions, objects):
    processed_ffts, labels = calculate_fft(data_path, actions[0], objects)
    act_lab = [actions[0]]*len(labels)
    
    p, l = calculate_fft(data_path, actions[1], objects)
    processed_ffts.extend(p)
    labels.extend(l)
    act_lab.extend([actions[1]]*len(l))
    
    p, l = calculate_fft(data_path, actions[2], objects)
    processed_ffts.extend(p)
    labels.extend(l)
    act_lab.extend([actions[2]]*len(l))
    
    p, l = calculate_fft(data_path, actions[3], objects)
    processed_ffts.extend(p)
    labels.extend(l)
    act_lab.extend([actions[3]]*len(l))

    #labels = act_lab
    #objects = actions
    print labels
    print 'len(processed_ffts)', len(processed_ffts)
    print 'len(labels)', len(labels)
    
    labels = np.asarray(labels)
    
    num_samplings = 50
    corr_arr = np.zeros(num_samplings)
    incorr_arr = np.zeros(num_samplings)
    confusion_matrix = np.zeros((len(objects), len(objects)), dtype=int)
    
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
        print train_labels
        print '*'*100
        print test_labels
        
        som, knn_model = train_model(train_set, train_labels)
        
        correct = 0
        for idx, seq in enumerate(test_set):
            label, probs = classify(seq, som, knn_model)
            pretty_print_knn_probs(test_labels[idx], label, probs)
            true_id = objects.index(test_labels[idx])
            pred_id = objects.index(label)
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
        
    print '  Correct mean/std/var', corr_arr.mean(), corr_arr.std(), corr_arr.var()
    print 'Incorrect mean/std/var', incorr_arr.mean(), incorr_arr.std(), incorr_arr.var()
    
    print_confusion_matrix(objects, confusion_matrix)
    
    for idx, obj in enumerate(objects):
        print '%4d --- %s' % (idx, obj)

from pybrain.datasets import UnsupervisedDataSet
from pybrain.unsupervised.trainers.deepbelief import DeepBeliefTrainer
from pybrain.tools.shortcuts import buildNetwork
from pybrain.structure import TanhLayer, SigmoidLayer, SoftmaxLayer

def run_dbn(data_path, actions, objects):
    processed_ffts, labels = calculate_fft(data_path, actions[0], objects)
    act_lab = [actions[0]]*len(labels)
    
#    p, l = calculate_fft(data_path, actions[1], objects)
#    processed_ffts.extend(p)
#    labels.extend(l)
#    act_lab.extend([actions[1]]*len(l))
#    
#    p, l = calculate_fft(data_path, actions[2], objects)
#    processed_ffts.extend(p)
#    labels.extend(l)
#    act_lab.extend([actions[2]]*len(l))
#    
#    p, l = calculate_fft(data_path, actions[3], objects)
#    processed_ffts.extend(p)
#    labels.extend(l)
#    act_lab.extend([actions[3]]*len(l))

    #labels = act_lab
    #objects = actions
    
    print labels
    print 'len(processed_ffts)', len(processed_ffts)
    print 'len(labels)', len(labels)
    
    labels = np.asarray(labels)
    
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
    print train_labels
    print '*'*100
    print test_labels
    
    ds = UnsupervisedDataSet(33)
    column_vectors = np.hstack(train_set).T
    np.random.shuffle(column_vectors)
    
    for vec in column_vectors:
        ds.addSample(vec)
    
    print len(ds)
    
    net = buildNetwork(33, 100, 10, hiddenclass=TanhLayer, outclass=SoftmaxLayer, bias=True)
    trainer = DeepBeliefTrainer(net, ds)
    print 'Training DBN net'
    trainer.train()
    print 'Done'
    #som, knn_model = train_model(train_set, train_labels)


#def run_multi_action(data_path, actions, objects):
#    action_models = {}
#    action_test_sets = {}
#    action_test_labels = {}
#    
#    for action in actions:
#        print 'Training action: %s' % action
#        processed_ffts, labels = calculate_fft(data_path, action, objects)
#        
#        inds = range(len(processed_ffts))
#        np.random.shuffle(inds)

#        num_tot = len(processed_ffts)
#        num_train = int(0.8 * num_tot)
#        num_test = num_tot - num_train
#        print 'Number of training instances', num_train
#        print 'Number of testing instances', num_test
#        
#        train_set = [processed_ffts[idx] for idx in inds[:num_train]]
#        train_labels = labels[inds][:num_train]
#        
#        action_models[action] = train_model(train_set, train_labels)
#        action_test_sets[action] = [processed_ffts[idx] for idx in inds[num_train:]]
#        action_test_labels[action] = labels[inds][num_train:]
#        
#    correct = 0
#    
#    for model in action_models:
#        
#        
#    for idx, seq in enumerate(test_set):
#        label, probs = classify(seq, som, knn_model)
#        pretty_print_knn_probs(test_labels[idx], label, probs)
#        true_id = objects.index(test_labels[idx])
#        pred_id = objects.index(label)
#        confusion_matrix[true_id][pred_id] += 1
#        if label == test_labels[idx]: correct += 1
#    pct_correct = float(correct) / len(test_set)
#    print 'Result: %.2f%% correct, %.2f%% incorrect' % (pct_correct*100, (1.0-pct_correct)*100)
#    
#    corr_arr[sampling] = pct_correct
#    incorr_arr[sampling] = 1.0-pct_correct
#    corr_sub = corr_arr[:sampling+1]
#    incorr_sub = incorr_arr[:sampling+1]
#    print 'Current correct mean/std/var   %.2f %.2f %.3f' % (corr_sub.mean(), corr_sub.std(), corr_sub.var())
#    print 'Current incorrect mean/std/var %.2f %.2f %.3f\n' % (incorr_sub.mean(), incorr_sub.std(), incorr_sub.var())

def seq_actions(data_path, actions, objects):
    processed_ffts, labels = calculate_fft(data_path, actions[0], objects)
    act_lab = [actions[0]]*len(labels)
    
    p, l = calculate_fft(data_path, actions[1], objects)
    processed_ffts.extend(p)
    labels.extend(l)
    act_lab.extend([actions[1]]*len(l))
    
    p, l = calculate_fft(data_path, actions[2], objects)
    processed_ffts.extend(p)
    labels.extend(l)
    act_lab.extend([actions[2]]*len(l))
    
    p, l = calculate_fft(data_path, actions[3], objects)
    processed_ffts.extend(p)
    labels.extend(l)
    act_lab.extend([actions[3]]*len(l))

    #labels = act_lab
    #objects = actions
    #print labels
    print 'len(processed_ffts)', len(processed_ffts)
    print 'len(labels)', len(labels)
    
    labels = np.asarray(labels)
    act_lab = np.asarray(act_lab)

    obj_inds = {}
    for obj in objects:
        obj_inds[obj] = np.where(labels == obj)[0]
        
    act_inds = {}
    for act in actions:
        act_inds[act] = np.where(act_lab == act)[0]
        
#    test_obj = 'clear_plastic_cup'
#    test_act = 'push'
#    a = obj_inds[test_obj]
#    b = act_inds[test_act]
#    
#    print a
#    print b
#    print a.shape
#    print b.shape
#    it = np.intersect1d(a,b)
#    print it 
#    print labels[it]
#    print act_lab[it]
#    exit(0)
    
    
    num_samplings = 5
    corr_arr = np.zeros(num_samplings)
    incorr_arr = np.zeros(num_samplings)
    confusion_matrix = np.zeros((len(objects), len(objects)), dtype=int)

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
        print train_labels
        print '*'*100
        print test_labels
        
        som, knn_model = train_model(train_set, train_labels)
        
        correct = 0
        total = 0
        for test_obj in test_labels:#objects:
            print 'Testing', test_obj
            inds1 = []
            acts = np.asarray(actions)[np.random.randint(len(actions),size=1)]
            for act in acts:#actions:
                a = obj_inds[test_obj]
                b = act_inds[act]
                c = np.intersect1d(a,b)
                inds1.append(c[np.random.randint(c.shape[0])])
            print inds1
            s1 = [processed_ffts[idx] for idx in inds1]
            total_prob = None
            for idx, seq in enumerate(s1):
                label, probs = classify(seq, som, knn_model)
                #pretty_print_knn_probs(test_obj, label, probs)
                
                # normalize, get rid of 0s
                for (k,v) in probs.items():
                    if v == 0: probs[k] = 0.001
                tot = sum(probs.values())
                for (k,v) in probs.items():
                    probs[k] = v / float(tot)
                    
                if not total_prob:
                    total_prob = probs
                else:
                    for (k,v) in probs.items():
                        total_prob[k] *= v #+= v
                    tot = sum(total_prob.values())
                    for (k,v) in total_prob.items():
                        total_prob[k] = v / float(tot)
                    
                sorted_probs = sorted(total_prob.iteritems(), key=itemgetter(1), reverse=True)[:5]
                
                print 'Probabilities after', idx+1, 'actions'
                for label, prob in sorted_probs:
                    print '%.4f --- %s' % (prob, label)
                    
            print '\n'
            sorted_probs = sorted(total_prob.iteritems(), key=itemgetter(1), reverse=True)[0]
            true_id = objects.index(test_obj)
            pred_id = objects.index(sorted_probs[0])
            confusion_matrix[true_id][pred_id] += 1
            if true_id == pred_id: correct += 1
            total += 1
            
        pct_correct = float(correct) / total
        print 'Result: %.2f%% correct, %.2f%% incorrect' % (pct_correct*100, (1.0-pct_correct)*100)

    print_confusion_matrix(objects, confusion_matrix)
    
    for idx, obj in enumerate(objects):
        print '%4d --- %s' % (idx, obj)

#        correct = 0
#        for idx, seq in enumerate(test_set):
#            label, probs = classify(seq, som, knn_model)
#            pretty_print_knn_probs(test_labels[idx], label, probs)
#            true_id = objects.index(test_labels[idx])
#            pred_id = objects.index(label)
#            confusion_matrix[true_id][pred_id] += 1
#            if label == test_labels[idx]: correct += 1
#        pct_correct = float(correct) / len(test_set)
#        print 'Result: %.2f%% correct, %.2f%% incorrect' % (pct_correct*100, (1.0-pct_correct)*100)
#        
#        corr_arr[sampling] = pct_correct
#        incorr_arr[sampling] = 1.0-pct_correct
#        corr_sub = corr_arr[:sampling+1]
#        incorr_sub = incorr_arr[:sampling+1]
#        print 'Current correct mean/std/var   %.2f %.2f %.3f' % (corr_sub.mean(), corr_sub.std(), corr_sub.var())
#        print 'Current incorrect mean/std/var %.2f %.2f %.3f\n' % (incorr_sub.mean(), incorr_sub.std(), incorr_sub.var())
#        
#    print '  Correct mean/std/var', corr_arr.mean(), corr_arr.std(), corr_arr.var()
#    print 'Incorrect mean/std/var', incorr_arr.mean(), incorr_arr.std(), incorr_arr.var()
#    
#    print_confusion_matrix(objects, confusion_matrix)
#    
#    for idx, obj in enumerate(objects):
#        print '%4d --- %s' % (idx, obj)


if __name__ == '__main__':
    #matrix()
    #exit(1)
    
    actions = ('drop', 'push', 'tap', 'shake')
    
    objects = ('pen',                       # 0
               'glasses',                   # 1
               #'terminals',                 # 2
               'toy_car',                   # 3
               'squeaky_ball',              # 4
               'empty_plastic_bottle',      # 5
               'nerf_basketball',           # 6
               'full_plastic_bottle',       # 7
               'clear_plastic_cup',         # 8
               'chickfila_sauce',           # 9
               'camcar_scew_box',           # 10
               'wire_spool',                # 11
               'aluminum_rod',              # 12
              )
              
    data_path = '/tmp/sounds'
    
    #run_batch_training_classification(data_path, actions, objects)
    #test_affinity(data_path, actions, objects)
    #run_dbn(data_path, actions, objects)
    seq_actions(data_path, actions, objects)

