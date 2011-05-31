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

import pickle

import numpy as np

from scipy.special import psi
from scipy.special import gamma
from scipy.special import gammaln
from numpy.random.mtrand import dirichlet

import roslib; roslib.load_manifest('ua_audio_infomax')
from ua_audio_infomax.msg import Action as InfomaxAction

from pybrain.rl.environments import EpisodicTask
from pybrain.utilities import Named
from pybrain.utilities import drawGibbs


__author__ = 'Daniel Ford, Antons Rebguns'
__copyright__ = 'Copyright (c) 2011 Daniel Ford, Antons Rebguns, Ian Fasel'

__license__ = 'BSD'
__maintainer__ = 'Daniel Ford'
__email__ = 'dford@email.arizona.edu'


class InfoMaxTask(EpisodicTask, Named):
    def __init__(self, environment,
                       sort_beliefs=True,
                       do_decay_beliefs=True,
                       uniform_initial_beliefs=True,
                       max_steps=30):
        EpisodicTask.__init__(self, environment)
        self.verbose = False
        self.listActions = False
        
        self.env = environment
        
        self.uniform_initial_beliefs = uniform_initial_beliefs
        self.max_steps = max_steps
        self.rewardscale = 1.0 #/self.max_steps
        
        self.state_ids = {'init': 0, 'grasped': 1, 'lifted': 2, 'placed': 3}
        
        self.initialize()


    def initialize(self, randomize=True):
        self.steps = 0
        self.current_location = 0
        
        alphas_pkl = open('/tmp/alphas.pkl', 'rb')
        alphas_map = pickle.load(alphas_pkl)
        alphas_pkl.close()
        
        self.prior_alphas = np.ones((len(alphas_map),self.env.num_categories,self.env.num_categories), dtype=float)
        
        for action_name in self.env.action_names:
            action_id = self.env.action_names.index(action_name)
            
            if action_name not in alphas_map: continue
            
            for category_name in self.env.category_names:
                category_id = self.env.category_names.index(category_name)
                self.prior_alphas[action_id,category_id] = alphas_map[action_name][category_name]
                
        self.env.reset(randomize)
        #print 'true objects', self.env.objects
        self.objects = [BeliefObjectDirichlet(self.env.num_categories, len(self.env.action_names), self.prior_alphas) for _ in self.env.objects]
        self.action_counts = np.zeros((len(self.objects),len(self.env.action_names)))
        self.state_counts = np.zeros((len(self.objects),len(self.state_ids.keys())))
        
        self.reward = 0.0
        self.percent_correct = 0.0
        self.maxentropy = self.calculate_entropy()
        self.prev_entropy = 0#self.maxentropy
        self.initialize_RBFs()


    def initialize_RBFs(self):
        """
        create RBFs to encode time to completion
        """
        self.numRBFs = 6
        self.sigma = self.max_steps / self.numRBFs
        self.RBFcenters = np.linspace(0, self.max_steps, self.numRBFs)    # will space centers in reals, need to cast to ints
        self.RBFs = np.zeros(self.numRBFs)


    def update_RBFs(self):
        for rbf in range(self.numRBFs):
            self.RBFs[rbf] = np.exp(-((self.steps - int(self.RBFcenters[rbf]))**2) / self.sigma)


    def update_beliefs(self, sensors, action):
        """
        update beliefs on each object, then compose individual beliefs into the
        full array
        """
        self.current_location = sensors.location
        
        # send PDF from sensor to update object PDF
        self.objects[sensors.location].update_alphas(action, sensors.beliefs)


    # calculate entropy of beliefs
    def calculate_entropy(self):
        ent = 0
        
        for obj in self.objects:
            ent += obj.compute_joint_entropy()
            
        return ent


    def reset(self, randomize=True):
        #EpisodicTask.reset(self)
        self.cumreward = 0
        self.samples = 0
        self.initialize(randomize)


    def getObservation(self):
        """
        returns current belief vector
        """
        beliefs = [self.objects[self.current_location].category_alphas]
        beliefs.extend([obj.category_alphas for obj in self.objects[self.current_location+1:]])
        beliefs.extend([obj.category_alphas for obj in self.objects[0:self.current_location]])
        beliefs = np.asarray(beliefs)
        
        curj = [self.objects[self.current_location].joint_prob]
        curj.extend([obj.joint_prob for obj in self.objects[self.current_location+1:]])
        curj.extend([obj.joint_prob for obj in self.objects[0:self.current_location]])
        curj = np.asarray(curj)
        
        act_count = [self.objects[self.current_location].action_count]
        act_count.extend([obj.action_count for obj in self.objects[self.current_location+1:]])
        act_count.extend([obj.action_count for obj in self.objects[0:self.current_location]])
        act_count = np.asarray(act_count)
        
        # update RBF activations
        self.update_RBFs()
        
        return np.concatenate((curj.flatten(),act_count.flatten(),self.RBFs))


    def performAction(self, action):
        """
        send chosen task to environment to be performed
        """
        if type(action) == np.ndarray:
            # Take the max, or random among them if there are several equal maxima
            action = drawGibbs(action, temperature=0)
            
        self.steps += 1
        #Important to note that we immediately update beliefs after performing action
        # (maybe not needed but it would make credit-assignment harder) """
        #self.current_location = self.env.performAction(action)
        
        # Get sensors and do belief updates before calling reward
        location_beliefs = self.env.sense(action)
        
        self.action_counts[self.env.current_location,action] += 1 #min(6, self.action_counts[self.env.current_location,action]+1)
        self.state_counts[self.env.current_location,self.state_ids[self.env.current_state]] += 1
        
        if location_beliefs.success:
            #print 'selected action: %s' % self.env.action_names[action]
            
            self.success = True
            self.update_beliefs(location_beliefs, action)
        else:
            self.success = False
            
        #predictions = np.array([np.argmax(obj.joint_prob) for obj in self.objects])
        #pred_correct = (predictions == self.env.objects)
        #self.percent_correct = pred_correct.sum() / float(pred_correct.size)
        #self.percent_correct /= (self.samples+1)
        self.percent_correct = np.array([self.objects[idx].joint_prob[obj_id] for idx,obj_id in enumerate(self.env.objects)]).sum() / len(self.objects)
        
        # Below two lines were cut and pasted from EpisodicTask.performAction()
        self.updateReward()
        self.addReward()
        self.samples += 1


    def isFinished(self):
        """
        set task completion criteria:
        max info reached, max number of actions performed, or failure condition
        """
        if self.steps >= self.max_steps: return True
        else: return False

    def getReward(self):
        return self.reward


    def updateReward(self):
        """
        compute and return the current reward (i.e. corresponding to the last
        action performed)
        """
        entropy = self.calculate_entropy() # less = more certain
        norm_ent = (self.maxentropy - entropy)/self.maxentropy
#        norm_ent = (entropy - self.maxentropy)/self.maxentropy
        norm_change = norm_ent - self.prev_entropy#entropy
#        print self.maxentropy, self.prev_entropy, entropy, norm_change
        
        self.prev_entropy = norm_ent#entropy
#        
#        # action entropy
#        n = float(len(self.objects))
#        counts = np.array([(obj.action_count / n).sum() for obj in self.objects])
#        tot_acts = counts.sum()
#        
#        u = np.ones(len(self.objects)) * (1.0/tot_acts)
#        u += 1.0e-5
#        u /= u.sum()
#        max_c_rew = -(u * np.log(u)).sum() * tot_acts
#        
#        if tot_acts == 0:
#            c_rew = 0.0
#        else:
#            counts += 1.0e-5
#            counts /= counts.sum()
#            c_rew = -(counts * np.log(counts)).sum()
#            c_rew *= tot_acts
#            #if 1.0 - (max_c_rew - c_rew) / max_c_rew < 0: print '************** LESS ************   ', max_c_rew, c_rew, 1.0 - (max_c_rew - c_rew) / max_c_rew
#            #c_rew = 1.0 - (max_c_rew - c_rew) / max_c_rew
            
        #print norm_ent, norm_change, c_rew
#        return norm_ent + norm_change
        self.reward = norm_ent * 2 + norm_change #self.percent_correct #+ norm_ent + # + norm_change#+ c_rew/50.0


    @property
    def indim(self):
        return len(self.env.action_names)     # number of actions we can take (minus reset)


    @property
    def outdim(self):
        return len(self.getObservation())     # belief vector


class BeliefObjectDirichlet(Named):
    def __init__(self, num_categories, num_actions, prior_alphas):
        self.num_categories = num_categories    # number of object categories
        self.num_actions = num_actions          # number of possible actions
        self.prior_alphas = prior_alphas
        self.reset()


    def reset(self):
        """
        resets all object probabilities
        """
        self.action_count = np.zeros(self.num_actions)
        #self.category_alphas = np.ones((self.num_actions,self.num_categories), dtype='float64') * 0.1
        self.category_alphas = np.ones((self.num_actions,self.num_categories), dtype=float) * (1.0/self.num_categories)
        
        self.update_joint_alphas()


    def update_dirichlet(self, prior, distribution, N=1):
        posterior = prior + distribution*N
        return posterior


    def computeJointAlpha(self, alphas):
        joint = []
        if True:
            for alpha in alphas:
                joint.append(alpha * (1.0/alpha.sum()) )
            joint = np.mean(np.array(joint),axis=0)
        else:
            joint = np.array(alphas).sum(axis=0)
        return joint


    def update_joint_alphas(self):
        self.joint_alphas = self.computeJointAlpha(self.category_alphas)
        self.joint_prob = self.joint_alphas / self.joint_alphas.sum()


    def update_alphas(self, action_id, current_pdf):
        # after moving current pdf is []
        if not current_pdf: return
        
        current_pdf = np.asarray(current_pdf)
        
        # get alphas over all objects for a given action (num_cat X num_cat matrix) and renormalize
        action_alphas = self.prior_alphas[action_id]
        
        def compute_prob(pdf, alphas):
            top = gammaln(alphas).sum()
            bottom = gammaln(alphas.sum())
            B = top - bottom
            mult = ((alphas-1.0) * np.log(current_pdf)).sum()
            return np.exp(mult - B)
            
        r = np.zeros(self.num_categories)
        for category in range(self.num_categories):
            r[category] = compute_prob(current_pdf, action_alphas[category])
            
        ps = self.category_alphas[action_id] * r
        posterior = ps / ps.sum()
        
        self.category_alphas[action_id] = posterior
        #self.category_alphas[action_id] = self.update_dirichlet(self.category_alphas[action_id], np.asarray(current_pdf))
        
        self.action_count[action_id] += 1
        self.update_joint_alphas()


    def computeB(self, alphas):
        top = gammaln(alphas).sum()
        bottom = gammaln(alphas.sum())
        return np.exp(top - bottom)


    def computeEntropy(self, alphas):
#        K = alphas.size
#        B = self.computeB(alphas)
#        a0 = alphas.sum()
#        H = np.log(B) + (a0 - K)*psi(a0) - ((alphas-1)*psi(alphas)).sum()
        p = alphas / alphas.sum()
        H = -(p * np.log(p)).sum()
#        print 'H', H
        return H


    def compute_joint_entropy(self):
        return self.computeEntropy(self.joint_alphas)

