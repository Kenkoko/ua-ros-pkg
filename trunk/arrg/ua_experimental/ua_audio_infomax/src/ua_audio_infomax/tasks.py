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


import numpy as np

from pybrain.rl.environments import EpisodicTask
from pybrain.utilities import Named
from pybrain.utilities import drawGibbs


__author__ = 'Daniel Ford, Antons Rebguns'
__copyright__ = 'Copyright (c) 2011 Daniel Ford, Antons Rebguns'
__credits__ = 'Ian Fasel'

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
        
        self.initialize()


    def initialize(self):
        self.steps = 0
        self.current_location = 0
        self.beliefs_are_new = True
        
        self.env.reset()
        self.objects = [BeliefObject(self.env.num_categories, len(self.env.action_names)) for _ in self.env.objects]
        
        self.initialize_beliefs()
        self.initialize_RBFs()


    def initialize_uniform_beliefs(self):
        self.beliefs = np.array([obj.category_probs for obj in self.objects])


    def initialize_beliefs(self):
        """
        initialize beliefs and entropy
        """
        self.initialize_uniform_beliefs()
        self.maxentropy = self.entropy()


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
        self.objects[sensors.location].update_probs(action, sensors.beliefs)
        
        # construct belief list from individual object PDFs
        self.beliefs = np.array([obj.category_probs for obj in self.objects])


    # calculate entropy of beliefs
    def entropy(self):
        ent = 0
        
        for obj_belief in self.beliefs:
            ent -= (np.log(obj_belief) * obj_belief).sum()
            
        return ent


    def reset(self):
        EpisodicTask.reset(self)
        self.initialize()


    def getObservation(self):
        """
        returns current belief vector
        """
        # sort object PDFs so the data for the object at our current location is first
        currLoc = self.beliefs[self.current_location:self.current_location+1]        # extract conditional PDFs for current object
        otherLocPre = self.beliefs[0:self.current_location]            # slice out PDFs before current object
        otherLocPost = self.beliefs[self.current_location+1:]            # slice out PDFs after current object
        beliefs = np.concatenate((currLoc,otherLocPost,otherLocPre))
        
        # update RBF activations
        self.update_RBFs()
        
        # otherwise, chop off old activations
#        if self.beliefs_are_new == False:
#            self.beliefs = self.beliefs[:self.env.num_categories*len(self.env.action_names)]
#        # if this is the first step in the episode, just set that flag
#        elif self.beliefs_are_new:
#            self.beliefs_are_new = False
            
        # append RBF activations to self.beliefs
        return np.concatenate((beliefs.flatten(),self.RBFs))


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
        self.update_beliefs(location_beliefs, action)
        
        # Below two lines were cut and pasted from EpisodicTask.performAction()
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
        """
        compute and return the current reward (i.e. corresponding to the last
        action performed)
        """
        return (self.maxentropy - self.entropy()) / self.maxentropy


    @property
    def indim(self):
        return len(self.env.action_names)     # number of actions we can take (minus reset)


    @property
    def outdim(self):
        return len(self.getObservation())     # belief vector


#####################################################################################
# class definitions for objects in the world and object properties that can be sensed
class BeliefObject(Named):
    def __init__(self, num_categories, num_actions, prob_calc_type='average'):
        self.num_categories = num_categories    # number of object categories
        self.num_actions = num_actions          # number of possible actions
        self.prob_calc_type = prob_calc_type
        
        self.action_count = np.zeros(self.num_actions)
        self._initialize_uniform_beliefs()      # PDF over object names


    def _initialize_uniform_beliefs(self):
        """
        initialize probabilities to be uniform
        """
        self.category_probs = np.tile(1.0 / self.num_categories, (self.num_actions,self.num_categories))
        self._update_joint_probs()


    def _update_joint_probs(self):
        # create joint probabilities from conditional probs
        if self.prob_calc_type == 'product':
            self.joint_prob = np.ones(self.num_categories)
            
            for cond_prob in self.category_probs:
                self.joint_prob *= cond_prob
                
        elif self.prob_calc_type == 'average':
            self.joint_prob = (self.action_count + 1) * np.asmatrix(self.category_probs)
            self.joint_prob = np.asarray(self.joint_prob[0])
            self.joint_prob = np.reshape(self.joint_prob, self.joint_prob.shape[1])
#            print type(self.action_count), self.action_count.shape
#            print type(self.joint_prob), self.joint_prob.shape
#            print 'action count', self.action_count
#            print 'category_probs', self.category_probs
#            print 'joint prob', self.joint_prob
            
        # normalize
        self.joint_prob /= self.joint_prob.sum()
#        print 'after divide', self.joint_prob


    def update_probs(self, action_id, current_pdf):
        """
        update conditional probabilities and joint probability
        takes the type of the last sensing action performed (integer) and the
        element of the returned PDF corresponding to that action type (float) 
        if we sensed instead of moved or reset, also update beliefs
        """
        if not current_pdf: return
        
        self.category_probs[action_id] *= self.action_count[action_id]
        self.category_probs[action_id] += current_pdf
        self.category_probs[action_id] /= self.action_count[action_id] + 1
        
        self.action_count[action_id] += 1
        self._update_joint_probs()


    def reset(self):
        """
        resets all object probabilities
        """
        self._initialize_uniform_beliefs()
        self.action_count = np.zeros(self.num_actions)


if __name__ == "__main__":
    # just some code for testing...
    PDF1 = [0.9, 0.05, 0.05]
    PDF2 = [0.12, 0.71, 0.17]
    
    cats = 3
    actions = 4
    objs = 1
    
    actionType = 3
    
    testObject0 = BeliefObject("Obj 1", cats, actions, objs)
    
    print "Obj 0"
    print testObject0.objProbs
    
    testObject0.update_probs(actionType,PDF1)
    print testObject0.actionCount[actionType]
    
    print "Obj 0"
    print testObject0.objProbs
    
    actionType = 1
    
    testObject0.update_probs(actionType,PDF2)
    print testObject0.actionCount[actionType]
    
    print "Obj 0"
    print testObject0.objProbs
    print testObject0.jointProb

