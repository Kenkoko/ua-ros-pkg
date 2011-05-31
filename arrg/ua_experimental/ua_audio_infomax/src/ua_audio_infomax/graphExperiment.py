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
import itertools

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import numpy as np

from scipy import stats


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
        
        #self.params = pickle.load(pkl)


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
        
        pkl_path = os.path.join(self.path, 'RewardsPerStep-trained.pkl')
        pkl = open(pkl_path)
        learned_rewards = np.array(pickle.load(pkl))
        handcoded_rewards = np.array(pickle.load(pkl))
        pkl.close()
        
        hand_reward = handcoded_rewards.sum(axis=1).mean()
        
        x_vals = np.arange(np.size(lrn_rewards, axis=1)) * (self.batch)
        means = np.mean(lrn_rewards, axis=0)
        stds = stats.sem(lrn_rewards, axis=0)
        
        fig = plt.figure()
        plt.plot(x_vals, means, 'r-')
        plt.plot([hand_reward]*len(x_vals), 'g-')
        plt.xlim(0, self.prnts * self.batch)
        plt.xlabel('Episode #')
        plt.ylabel('Reward(-H)')
        
        plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor='red')
        plt.savefig(os.path.join(fig_path, 'RewardsPerEpisode-learning.pdf'))


    def plot_rewards_per_trained_step(self, fig_path):
        pkl_path = os.path.join(self.path, 'RewardsPerStep-trained.pkl')
        pkl = open(pkl_path)
        learned_rewards = np.array(pickle.load(pkl))
        handcoded_rewards = np.array(pickle.load(pkl))
        pkl.close()
        
        pad = np.zeros((learned_rewards.shape[0],1))
        learned_rewards = np.hstack((pad,learned_rewards))
        handcoded_rewards = np.hstack((pad,handcoded_rewards))
        
        x_vals = np.arange(np.size(learned_rewards, axis=1))
        means = np.mean(learned_rewards, axis=0)
        stds = stats.sem(learned_rewards, axis=0)
        
        means2 = np.mean(handcoded_rewards, axis=0)
        stds2 = stats.sem(handcoded_rewards, axis=0)
        
        fig = plt.figure()
        fig.set_figheight(2.50)
        plt.plot(x_vals, means, 'r+-')
        plt.plot(x_vals, means2, 'gx-')
        plt.xlim(0, self.maxSteps)
        plt.xlabel('Step #')
        plt.ylabel('Reward(-H)')
        plt.legend( ('trained policy', 'hand-coded policy'), loc='best')
        
        plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor='red')
        plt.fill_between(x_vals, means2+stds2, y2=means2-stds2, alpha=0.2, facecolor='green')
        plt.savefig(os.path.join(fig_path, 'RewardsPerStep-trained.pdf'))
        
        # cumulative reward graph
        cumulative_reward_learned = means.cumsum()
        cumulative_reward_handcoded = means2.cumsum()
        
        plt.figure()
        plt.plot(x_vals, cumulative_reward_learned, 'r+-')
        plt.plot(x_vals, cumulative_reward_handcoded, 'gx-')
        plt.xlabel('Step #')
        plt.ylabel('Cumulative Reward(-H)')
        plt.legend( ('trained policy', 'hand-coded policy'), loc='best')
        
        plt.savefig(os.path.join(fig_path, 'CumulativeRewardsPerStep.pdf'))
        
        # new cumsum graph
        sums = np.cumsum(learned_rewards, axis=1)
        means = np.mean(sums, axis=0)
        stds = np.std(sums, axis=0) #stats.sem(sums, axis=0)
        
        sums2 = np.cumsum(handcoded_rewards, axis=1)
        means2 = np.mean(sums2, axis=0)
        stds2 = np.std(sums2, axis=0) #stats.sem(sums2, axis=0)
        
        plt.figure()
        plt.plot(x_vals, means, 'r+-')
        plt.plot(x_vals, means2, 'gx-')
        plt.xlim(0, self.maxSteps)
        plt.xlabel('Step #')
        plt.ylabel('Cumulative Reward(-H)')
        plt.legend( ('trained policy', 'hand-coded policy'), loc='best')
        
        plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor='red')
        plt.fill_between(x_vals, means2+stds2, y2=means2-stds2, alpha=0.2, facecolor='green')
        plt.savefig(os.path.join(fig_path, 'CumulativeRewardsPerStep_stddev.pdf'))


    def plot_accuracy_per_step(self, fig_path):
        pkl_path = os.path.join(self.path, 'AccuracyPerStep.pkl')
        pkl = open(pkl_path)
        probCorrect = np.array(pickle.load(pkl), dtype='float64')
        probCorrectHand = np.array(pickle.load(pkl), dtype='float64')
        pkl.close()
        
        pad = np.zeros((probCorrect.shape[0],probCorrect.shape[1],1))
        probCorrect = np.dstack((pad,probCorrect))
        probCorrectHand = np.dstack((pad,probCorrectHand))
        
        x_vals = np.arange(np.size(probCorrect, axis=2))
        
        # average across all test episodes and then across all locations
        if probCorrect.shape[1] == 1: #one object case
            means = probCorrect.mean(axis=0).flatten()
            stds = stats.sem(probCorrect, axis=0).flatten()
            
            means2 = probCorrectHand.mean(axis=0).flatten()
            stds2 = stats.sem(probCorrectHand, axis=0).flatten()
        else:
            means = probCorrect.mean(axis=0).mean(axis=0)
            stds = stats.sem(stats.sem(probCorrect, axis=0), axis=0)
            
            means2 = probCorrectHand.mean(axis=0).mean(axis=0)
            stds2 = stats.sem(stats.sem(probCorrectHand, axis=0), axis=0)
        
        plt.figure()
        plt.plot(x_vals, means, 'r+-')
        
        plt.plot(x_vals, means2, 'gx-')
        plt.axis([0, self.maxSteps, 0, 1.10])
        plt.xlabel('Step #')
        plt.ylabel('Accuracy (%)')
        plt.legend( ('trained policy', 'hand-coded policy'), loc='best')
        
        plt.fill_between(x_vals, means+stds, y2=means-stds, alpha=0.2, facecolor='red')
        plt.fill_between(x_vals, means2+stds2, y2=means2-stds2, alpha=0.2, facecolor='green')
        plt.savefig(os.path.join(fig_path, 'AccuracyPerStep.pdf'))
        
        # dump in latex table form
        f = open(os.path.join(fig_path, 'accuracy_table.tex'), 'w')
        
        indices_40_steps = np.array([1,4,9,13,18,30,40])
        indices_25_steps = np.array([1,4,9,13,18,21,25])
        indices_15_steps = np.array([1,4,7, 9,11,13,15])
        
        print means.size
        if means.size >= 40: indices = indices_40_steps
        elif means.size >= 25: indices = indices_25_steps
        elif means.size >= 15: indices = indices_15_steps
        else: indices = np.arange(means.size)
        
        vals = {}
#        vals['tabular'] = '|l' + '|c' * means.size + '|'
#        vals['step_row'] = ' '.join(['& %d' % step for step in range(means.size)])
#        vals['learned_row'] = ' '.join(['& %2.1f' % (acc*100) for acc in means])
#        vals['handcoded_row'] = ' '.join(['& %2.1f' % (acc*100) for acc in means2])
        vals['tabular'] = '|l' + '|c' * indices.size + '|'
        vals['step_row'] = ' '.join(['& %d' % step for step in indices])
        vals['learned_row'] = ' '.join(['& %2.1f' % (acc*100) for acc in means[indices]])
        vals['handcoded_row'] = ' '.join(['& %2.1f' % (acc*100) for acc in means2[indices]])
        
        table = (r'''
\begin{table}[tbp]
\centering
\begin{tabular}{%(tabular)s}
\hline
Step Number %(step_row)s\\
\hline
Learned \%% %(learned_row)s\\
Handcoded \%% %(handcoded_row)s\\
\hline
\end{tabular}
\caption{Classification Accuracy per Step}
\label{tab:template}
\end{table}'''
         % vals)
        
        f.write(table)
        f.close()

    def plot_joint_probs_per_step(self, fig_path):
        pkl_path = os.path.join(self.path, 'JointProbsPerStep-learned.pkl')
        pkl = open(pkl_path)
        
        learned_steps = np.array(pickle.load(pkl))
        joint_probs_learned = np.array(pickle.load(pkl))
        
        handcoded_steps = pickle.load(pkl)
        joint_probs_handcoded = np.array(pickle.load(pkl))
        
        learned_true = np.array(pickle.load(pkl))
        handcoded_true = np.array(pickle.load(pkl))
        
        pkl.close()
        
        p = np.array(['']*learned_steps.shape[0])
        pad = np.reshape(p, (p.shape[0],-1))
        learned_steps = np.hstack((learned_steps,pad))
        handcoded_steps = np.hstack((handcoded_steps,pad))
        
        pad = np.ones((joint_probs_learned.shape[0],joint_probs_learned.shape[1],1,joint_probs_learned.shape[3])) / joint_probs_learned.shape[3]
        joint_probs_learned = np.dstack((pad,joint_probs_learned))
        joint_probs_handcoded = np.dstack((pad,joint_probs_handcoded))
        
        pkl_path = os.path.join(self.path, 'RewardsPerStep-trained.pkl')
        pkl = open(pkl_path)
        learned_rewards = np.array(pickle.load(pkl))
        handcoded_rewards = np.array(pickle.load(pkl))
        pkl.close()
        
        pad = np.zeros((learned_rewards.shape[0],1))
        learned_rewards = np.hstack((pad,learned_rewards))
        handcoded_rewards = np.hstack((pad,handcoded_rewards))
        
        matplotlib.rcParams['legend.fontsize'] = 6
        matplotlib.rcParams['xtick.labelsize'] = 5
        matplotlib.rcParams['ytick.labelsize'] = 6
        
        styles = itertools.cycle(['b+', 'gx', 'rd', 'co', 'mv', 'y^', 'k*', 'bx', 'g+', 'r*'])
        
        for test_run_idx in range(len(learned_steps)-1):
            fig1 = plt.figure()
            fig1.suptitle('Category Joint Probabilities (Learned)')
            fig1.set_figwidth(4.50)
            legend_present = False
            plt.subplots_adjust(hspace=0.001)
            
            for object_location,object_idx in enumerate(learned_true[test_run_idx]):
                ax = fig1.add_subplot(len(learned_true[test_run_idx])+1,1,object_location+1)
                ax.plot(joint_probs_learned[test_run_idx,object_location])
                ax.set_xticks(np.arange(len(learned_steps[test_run_idx])))
                max_val = np.round(np.max(joint_probs_learned[test_run_idx,object_location]), 1)
                ax.set_yticks(np.arange(0.0, max_val+0.05, 0.05))
                l = np.arange(0.05, max_val, 0.1).tolist()
                l = np.array(zip(['']*len(l), l)).flatten()
                ax.set_yticklabels(l)
                ax.set_xticklabels([])
                ax.get_xaxis().tick_bottom()
                #plt.xticks(np.arange(len(learned_steps[test_run_idx])), learned_steps[test_run_idx], rotation='vertical')
                
                #plt.axis([0, self.maxSteps, 0, 1])
                
                true_obj_name = self.object_names[object_idx]
                ax.yaxis.set_label_position('right')
                plt.ylabel(true_obj_name, size='x-small')
                
                if not legend_present:
#                    ax.set_title('Category Joint Probabilities (Learned)', size='x-small')
                    props = font_manager.FontProperties(size=5)
                    leg = plt.legend(self.object_names, bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=5, mode="expand", borderaxespad=0., fancybox=True, prop=props)
                    frame = leg.get_frame()
                    frame.set_facecolor('0.95')    # set the frame face color to light gray
                    legend_present = True
                    
            ax = fig1.add_subplot(len(learned_true[test_run_idx])+1,1,len(learned_true[test_run_idx])+1)
            ax.plot(learned_rewards[test_run_idx], 'r+-')
            plt.xticks(np.arange(len(learned_steps[test_run_idx])), learned_steps[test_run_idx], rotation='vertical')
            max_val = np.round(np.max(learned_rewards[test_run_idx]), 1)
            ax.set_yticks(np.arange(0.0, max_val+0.1, 0.05))
            ax.set_yticklabels(np.arange(0.0, max_val+0.05, 0.05))
            ax.yaxis.set_label_position('right')
            ax.get_xaxis().tick_bottom()
            
            plt.xlim(0, self.maxSteps)
            plt.ylabel('Reward(-H)', size='x-small')
            
            plt.savefig(os.path.join(fig_path, 'JointProbsPerStep-%s-learned-%d.pdf' % (str([self.object_names[i] for i in learned_true[test_run_idx]]), test_run_idx)))
            
        for test_run_idx in range(len(handcoded_steps)-1):
            fig1 = plt.figure()
            fig1.suptitle('Category Joint Probabilities (Handcoded)', size='small')
            fig1.set_figwidth(4.50)
            legend_present = False
            plt.subplots_adjust(hspace=0.001)
            
            for object_location,object_idx in enumerate(handcoded_true[test_run_idx]):
                ax = fig1.add_subplot(len(handcoded_true[test_run_idx])+1,1,object_location+1)
                ax.plot(joint_probs_handcoded[test_run_idx,object_location])
                ax.set_xticks(np.arange(len(handcoded_steps[test_run_idx])))
                max_val = np.round(np.max(joint_probs_handcoded[test_run_idx,object_location]), 1)
                ax.set_yticks(np.arange(0.0, max_val+0.05, 0.05))
                l = np.arange(0.05, max_val, 0.1).tolist()
                l = np.array(zip(['']*len(l), l)).flatten()
                ax.set_yticklabels(l)
                ax.set_xticklabels([])
                ax.get_xaxis().tick_bottom()
                #plt.xticks(np.arange(len(handcoded_steps[test_run_idx])), handcoded_steps[test_run_idx], rotation='vertical')
                
                #plt.axis([0, self.maxSteps, 0, 1])
                
                true_obj_name = self.object_names[object_idx]
                ax.yaxis.set_label_position('right')
                plt.ylabel(true_obj_name, size='x-small')
                
                if not legend_present:
#                    ax.set_title('Category Joint Probabilities (Handcoded)', size='x-small')
                    props = font_manager.FontProperties(size=5)
                    leg = plt.legend(self.object_names, bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=5, mode="expand", borderaxespad=0., fancybox=True, prop=props)
                    frame = leg.get_frame()
                    frame.set_facecolor('0.95')    # set the frame face color to light gray
                    legend_present = True
                    
            ax = fig1.add_subplot(len(handcoded_true[test_run_idx])+1,1,len(handcoded_true[test_run_idx])+1)
            ax.plot(handcoded_rewards[test_run_idx], 'r+-')
            plt.xticks(np.arange(len(handcoded_steps[test_run_idx])), handcoded_steps[test_run_idx], rotation='vertical')
            max_val = np.round(np.max(handcoded_rewards[test_run_idx]), 1)
            ax.set_yticks(np.arange(0.0, max_val+0.1, 0.05))
            ax.set_yticklabels(np.arange(0.0, max_val+0.05, 0.05))
            ax.yaxis.set_label_position('right')
            ax.get_xaxis().tick_bottom()
            
            plt.xlim(0, self.maxSteps)
            plt.ylabel('Reward(-H)', size='x-small')
            
            plt.savefig(os.path.join(fig_path, 'JointProbsPerStep-%s-handcoded-%d.pdf' % (str([self.object_names[i] for i in handcoded_true[test_run_idx]]), test_run_idx)))


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

