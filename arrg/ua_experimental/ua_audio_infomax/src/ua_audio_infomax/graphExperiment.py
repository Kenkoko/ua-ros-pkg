#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2011, Daniel Ford, Antons Rebguns
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# Neither the name of the <ORGANIZATION> nor the names of its contributors may
# be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os
import sys
import pickle
import datetime

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np


__author__ = 'Daniel Ford, Antons Rebguns'
__copyright__ = 'Copyright (c) 2011 Daniel Ford, Antons Rebguns'
__credits__ = 'Ian Fasel'

__license__ = 'BSD'
__maintainer__ = 'Daniel Ford'
__email__ = 'dford@email.arizona.edu'


class graph():
    def __init__(self, path):
        self.path = path
        pkl_path = os.path.join(self.path, 'experiment.desc')
        
        pkl = open(pkl_path)
        
        self.timestamp = pickle.load(pkl)
        self.numCategories = pickle.load(pkl)
        self.object_names = pickle.load(pkl)
        self.action_names = pickle.load(pkl)
        self.numbExp = pickle.load(pkl)
        self.prnts = pickle.load(pkl)
        self.batch = pickle.load(pkl)
        self.numTestingEps = pickle.load(pkl)
        self.numTestRunEps = pickle.load(pkl)
        self.maxSteps = pickle.load(pkl)
        
        self.params = pickle.load(pkl)


    def print_data(self):
        print ""
        print "***** experiment parameters *****"
        print ""
        print "timestamp:", self.timestamp
        print "categories:", self.numCategories
        print 'category names', self.object_names
        print 'action names:', self.action_names
        print ""
        print "experiments:", self.numbExp
        print "batches per experiment:", self.prnts
        print "episodes per batch:", self.batch
        print "testing episodes per batch:", self.numTestingEps
        print "episodes to run trained policy:", self.numTestRunEps
        print "steps per episode:", self.maxSteps
        print ""


    def plot_rewards_per_learning_episode(self, fig_path):
        pkl_path = os.path.join(self.path, 'RewardsPerEpisode-learning.pkl')
        pkl = open(pkl_path)
        lrn_rewards = pickle.load(pkl)
        pkl.close()
        
        x_vals = np.arange(np.size(lrn_rewards, axis=1)) * (self.batch)
        means = np.mean(lrn_rewards, axis=0)
        stds = np.std(lrn_rewards, axis=0)
        
        plt.figure()
        plt.plot(x_vals, means, 'gx-')
        plt.xlim(0, self.prnts * self.batch)
        plt.xlabel('Episode #')
        plt.ylabel('Reward(-H)')
        
        plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor='green')
        plt.savefig(os.path.join(fig_path, 'RewardsPerEpisode-learning.pdf'))


    def plot_rewards_per_trained_step(self, fig_path):
        pkl_path = os.path.join(self.path, 'RewardsPerStep-trained.pkl')
        pkl = open(pkl_path)
        learned_rewards = pickle.load(pkl)
        handcoded_rewards = pickle.load(pkl)
        pkl.close()
        
        x_vals = np.arange(np.size(learned_rewards, axis=1))
        means = np.mean(learned_rewards, axis=0)
        stds = np.std(learned_rewards, axis=0)
        
        means2 = np.mean(handcoded_rewards, axis=0)
        stds2 = np.std(handcoded_rewards, axis=0)
        
        plt.figure()
        plt.plot(x_vals, means, 'r+-')
        plt.plot(x_vals, means2, 'gx-')
        plt.xlim(0, self.maxSteps - 1)
        plt.xlabel('Step #')
        plt.ylabel('Reward(-H)')
        plt.legend( ('trained policy', 'hand-coded policy'), loc='best')
        
        plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor='red')
        plt.fill_between(x_vals, means2+stds2, y2=means2-stds2, alpha=0.2, facecolor='green')
        plt.savefig(os.path.join(fig_path, 'RewardsPerStep-trained.pdf'))


    def plot_accuracy_per_step(self, fig_path):
        pkl_path = os.path.join(self.path, 'AccuracyPerStep.pkl')
        pkl = open(pkl_path)
        probCorrect = pickle.load(pkl)
        probCorrectHand = pickle.load(pkl)
        pkl.close()
        
        x_vals = np.arange(np.size(probCorrect, axis=2))
        
        # average across all test episodes and then across all locations
        means = probCorrect.mean(axis=0).mean(axis=0)
        stds = probCorrect.std(axis=0).std(axis=0)
        
        means2 = probCorrectHand.mean(axis=0).mean(axis=0)
        stds2 = probCorrectHand.std(axis=0).std(axis=0)
        
        plt.figure()
        plt.plot(x_vals, means, 'r+-')
        plt.plot(x_vals, means2, 'gx-')
        plt.axis([0, self.maxSteps - 1, 0, 1.10])
        plt.xlabel('Step #')
        plt.ylabel('Accuracy (%)')
        plt.legend( ('trained policy', 'hand-coded policy'), loc='best')
        
        plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor='red')
        plt.fill_between(x_vals, means2+stds2, y2=means2-stds2, alpha=0.2, facecolor='green')
        plt.savefig(os.path.join(fig_path, 'AccuracyPerStep.pdf'))


    def plot_joint_probs_per_step(self, fig_path):
        pkl_path = os.path.join(self.path, 'JointProbsPerStep-learned.pkl')
        pkl = open(pkl_path)
        
        learned_steps = pickle.load(pkl)
        joint_probs_learned = pickle.load(pkl)
        
        handcoded_steps = pickle.load(pkl)
        joint_probs_handcoded = pickle.load(pkl)
        
        learned_true = pickle.load(pkl)
        handcoded_true = pickle.load(pkl)
        
        pkl.close()
        
        pkl_path = os.path.join(self.path, 'RewardsPerStep-trained.pkl')
        pkl = open(pkl_path)
        learned_rewards = pickle.load(pkl)
        handcoded_rewards = pickle.load(pkl)
        pkl.close()
        
        matplotlib.rcParams['legend.fontsize'] = 6
        matplotlib.rcParams['xtick.labelsize'] = 6
        matplotlib.rcParams['ytick.labelsize'] = 6
        
        for test_run_idx in range(len(learned_steps)):
            fig1 = plt.figure()
            legend_present = False
            
            for object_location,object_idx in enumerate(learned_true[test_run_idx]):
                ax = fig1.add_subplot(1,len(learned_true[test_run_idx])+1,object_location+1)
                ax.plot(joint_probs_learned[test_run_idx,object_location])
                #ax.xaxis.set_ticklabels(learned_steps[test_run_idx])
                #plt.xticks(rotation='vertical')
                plt.xticks(np.arange(len(learned_steps[test_run_idx])), learned_steps[test_run_idx], rotation='vertical')
                
                plt.axis([0, self.maxSteps - 1, 0, 1])
                
                true_obj_name = self.object_names[object_idx]
                ax.set_title(true_obj_name)
                
                #plt.xlabel('Step #')
                
                if not legend_present:
                    plt.ylabel('Category Joint Probabilities (Learned)')
                    plt.legend(self.object_names, 'best')
                    legend_present = True
                    
            ax = fig1.add_subplot(1,len(learned_true[test_run_idx])+1,len(learned_true[test_run_idx])+1)
            ax.plot(learned_rewards[test_run_idx], 'r+-')
            ax.xaxis.set_ticklabels(learned_steps[test_run_idx])
            plt.xticks(rotation='vertical')
            ax.yaxis.set_label_position('right')
            
            plt.xlim(0, self.maxSteps - 1)
            plt.xlabel('Step #')
            plt.ylabel('Reward(-H)')
            
            plt.savefig(os.path.join(fig_path, 'JointProbsPerStep-learned-%s-%d.pdf' % (str([self.object_names[i] for i in learned_true[test_run_idx]]), test_run_idx)))
            
        for test_run_idx in range(len(handcoded_steps)):
            fig1 = plt.figure()
            legend_present = False
            
            for object_location,object_idx in enumerate(handcoded_true[test_run_idx]):
                ax = fig1.add_subplot(1,len(handcoded_true[test_run_idx])+1,object_location+1)
                ax.plot(joint_probs_handcoded[test_run_idx,object_location])
                ax.xaxis.set_ticklabels(handcoded_steps[test_run_idx])
                plt.xticks(rotation='vertical')
                
                plt.axis([0, self.maxSteps - 1, 0, 1])
                
                true_obj_name = self.object_names[object_idx]
                ax.set_title(true_obj_name)
                
                #plt.xlabel('Step #')
                
                if not legend_present:
                    plt.ylabel('Category Joint Probabilities (Handcoded)')
                    plt.legend(self.object_names, 'best')
                    legend_present = True
                    
            ax = fig1.add_subplot(1,len(handcoded_true[test_run_idx])+1,len(handcoded_true[test_run_idx])+1)
            ax.plot(handcoded_rewards[test_run_idx], 'r+-')
            ax.xaxis.set_ticklabels(handcoded_steps[test_run_idx])
            plt.xticks(rotation='vertical')
            ax.yaxis.set_label_position('right')
            
            plt.xlim(0, self.maxSteps - 1)
            plt.xlabel('Step #')
            plt.ylabel('Reward(-H)')
            
            plt.savefig(os.path.join(fig_path, 'JointProbsPerStep-handcoded-%s-%d.pdf' % (str([self.object_names[i] for i in handcoded_true[test_run_idx]]), test_run_idx)))


    def plot_all(self):
        path = os.path.join('/tmp', datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S"))
        if not os.path.exists(path): os.makedirs(path)
        
        self.plot_rewards_per_learning_episode(path)
        self.plot_rewards_per_trained_step(path)
        self.plot_accuracy_per_step(path)
        self.plot_joint_probs_per_step(path)
        
        print 'ALL GRAPHS DONE'


if __name__ == '__main__':
        if len(sys.argv) < 3:
            path = ''
        else:
            path = sys.argv[1]
            
        graph1 = graph(path)
        graph1.print_data()
        graph1.plot_all()

