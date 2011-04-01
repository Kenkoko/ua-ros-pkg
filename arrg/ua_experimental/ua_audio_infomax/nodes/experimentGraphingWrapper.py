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


"""
wrapper for InfoMax reinforcement learning with PyBrain
runs InfoMax agent in batches and collects and plots data
"""


import os
import sys
import pickle
import datetime

from optparse import OptionParser

import numpy as np

import roslib; roslib.load_manifest('ua_audio_infomax')
import rospy

# import optimization frameworks and agent class
from pybrain.optimization import PGPE
from pybrain.rl.agents import OptimizationAgent
from pybrain.rl.experiments import EpisodicExperiment

# import neural net stuff
from pybrain.structure.networks.network import Network
from pybrain.tools.shortcuts import buildNetwork
from pybrain.structure.modules import SoftmaxLayer
from pybrain.utilities import drawGibbs

from ua_audio_infomax.tasks import InfoMaxTask
from ua_audio_infomax.environment import InfoMaxEnv
from ua_audio_infomax.graphExperiment import graph


__author__ = 'Daniel Ford, Antons Rebguns'
__copyright__ = 'Copyright (c) 2011 Daniel Ford, Antons Rebguns'
__credits__ = 'Ian Fasel'

__license__ = 'BSD'
__maintainer__ = 'Daniel Ford'
__email__ = 'dford@email.arizona.edu'


if __name__ == '__main__':
    usage_msg = 'Usage: %prog [options]'
    desc_msg = 'Runs InfoMax agent in batches and collects and plots data.'
    epi_msg = 'Example: %s --num-objects=5 --max-steps=35' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-o', '--num-objects', metavar='OBJ', type='int', default=1,
                      help='number of objects in the world [default: %default]')
    parser.add_option('-e', '--num-experiments', metavar='EXP', type='int', default=2,
                      help='number of learning experiments [default: %default]')
    parser.add_option('-b', '--num-batches', metavar='BATCH', type='int', default=10,
                      help='number of batches per experiment')
    parser.add_option('-l', '--num-learning-episodes', metavar='LEPI', type='int', default=100,
                      help='number of learning episodes per batch')
    parser.add_option('-t', '--num-testing-episodes', metavar='TEPI', type='int', default=10,
                      help='number of testing episodes per batch')
    parser.add_option('-r', '--num-best-test-runs', metavar='BRUNS', type='int', default=10,
                      help='number of episodes to run the best overall agent')
    parser.add_option('-m', '--max-steps', metavar='STEPS', type='int', default=10,
                      help='number of steps per episode')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    num_objects = options.num_objects
    num_experiments = options.num_experiments
    num_batches = options.num_batches
    num_learning_episodes = options.num_learning_episodes
    num_testing_episodes = options.num_testing_episodes
    num_best_test_runs = options.num_best_test_runs
    max_steps = options.max_steps
    
    # categories and objects 
    num_categories = 10
    
    action_names = ['grasp',        # 0
                    'lift',         # 1
                    'drop',         # 2
                    'shake_roll',   # 3
                    'place',        # 4
                    'push',         # 5
                    'shake_pitch',  # 6
                    'move_left',    # 7
                    'move_right',   # 8
                   ]
                   
    object_names = ['pink_glass',           # 0
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
                   
    # init structures for rewards and network parameters
    lrn_rewards = []
    best_params = []
    best_reward = -1000;
    
    # save off experiment parameters for each run
    def pickle_metadata(filename, timestamp, netparams):
        f = open(filename, 'w')
        
        pickle.dump(timestamp,f)                # timestamp
        pickle.dump(num_categories,f)           # number of categories
        pickle.dump(object_names,f)             # object name and category
        pickle.dump(action_names,f)             # action names
        
        pickle.dump(num_experiments,f)          # number of experiments
        pickle.dump(num_batches,f)              # number of learning batches per experiment
        pickle.dump(num_learning_episodes,f)    # number of episodes per num_learning_episodes
        pickle.dump(num_testing_episodes,f)     # number of testing episodes per num_learning_episodes
        pickle.dump(num_best_test_runs,f)       # number of episodes to run the best agent
        pickle.dump(max_steps,f)                # number of steps per episode 
        
        pickle.dump(netparams,f)                # parameters of trained policy
        
        f.close()
        
    rospy.init_node('experiment_graphing_node', anonymous=True)
    
    ###################### run [num_experiments] experiments, each with [num_batches] batches
    ###################### each num_learning_episodes has [num_learning_episodes] episodes with [max_steps] per episode 
    best_counter = 0
    
    for runs in range(num_experiments):
        rospy.loginfo('\n*********** STARTING EXPERIMENT %d ***********' % runs)
        
        # set up environment, task, neural net, agent, and experiment
        env = InfoMaxEnv(object_names, action_names, num_objects)
        task = InfoMaxTask(env, max_steps=max_steps)
        net = buildNetwork(task.outdim, task.indim, bias =True, outclass=SoftmaxLayer)
        agent = OptimizationAgent(net, PGPE(storeAllEvaluations=True,minimize=False,verbose=False))
        experiment = EpisodicExperiment(task, agent)
        
        agent_rewards = []
        ep_rewards = []
        
        # Learn in batches
        agent_rewards.append(0)
        
        for i in range(num_batches):
            rospy.loginfo('\tProcessing learning batch %d...' % i)
            
            ########################## learn policy
            task.env.listActions = True
            experiment.doEpisodes(num_learning_episodes)
            
            ########################## test learned policy
            
            # When a num_learning_episodes is done, evaluate so we can see progress and show learning curves
            curparams = agent.learner.current;
            agent.learner.wrappingEvaluable._setParameters(curparams);
            
            # Evaluate the current learned policy for num_testing_episodes episodes
            rewards = []
            
            for test_ep in range(num_testing_episodes):
#                    print "\t\tTESTING EP", test_ep
                agent.newEpisode()
                # Execute the agent in the environment without learning for one episode.
                # This uses the current set of parameters
                r = agent.learner._BlackBoxOptimizer__evaluator(agent.learner.wrappingEvaluable)
#                    print "\t\t\treward testing episode", test_ep, r
                rewards.append(r)
                #testingJointProbs.append(task.objects[0].jointProb)
                
            # average of all rewards earned by the current policy running num_testing_episodes episodes
            avg_testing_reward = np.mean(rewards)
            
            rospy.loginfo('\taverage testing reward for batch %d is [%.5f]' % (i, avg_testing_reward))
            
            # list of average rewards per num_learning_episodes 
            agent_rewards.append(avg_testing_reward)
            
            # list of reward per episode in this num_learning_episodes
            ep_rewards.append(rewards)
            
            # compare the average reward for this evaluation of the learned policy. If it is better on average than
            # a previous one, then save off the parameters that make up the neural network so we can use it to 
            # perform a single episode
            if avg_testing_reward > best_reward:
                best_counter = 0
                best_reward = avg_testing_reward
                bestparams = agent.learner.current.copy()
                
            rospy.loginfo('\tCurrent best reward is [%.5f] (unchanged for %d batches)' % (best_reward, best_counter))
            best_counter += 1
            
        # list of list of average rewards per num_learning_episodes
        lrn_rewards.append(agent_rewards)
        
        # list of lists of rewards per episode in each num_learning_episodes
        #lrn_rewards.append(ep_rewards)
        
        # saving parameters of the best policy
        best_params2 = bestparams.copy();
        # All Batches done.  
        
    rospy.loginfo('Learning DONE')
    
    ###################### set path to save experiment files and metadata
    path = ""
    #path = "data"
    #os.system("cd .."); os.system("chmod -R 777 data")    # brute-force method to ensure we have the right permissions
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    #path = "./" + path + "/" + timestamp + "/"
    #os.mkdir(path)
    filename = "experiment.desc"
    # save metadata, including trained policy parameters
    pickle_metadata(path+filename, timestamp, best_params2)
    
    ###################### save rewards per num_learning_episodes during learning
    outfile = path+"RewardsPerEpisode-learning.pkl"
    f = open(outfile, 'w')
    pickle.dump(lrn_rewards,f)
    f.close()
    
    ###################### now run the best agent
    # need to save action trace, also a good place to insert hand-coded policy for running
    Ep_rewards = []
    probCorrect = []
    Ep_rewardsHand = []
    probCorrectHand = []
    agent.learner.wrappingEvaluable._setParameters(best_params2)
    
    joint_probs_learned = np.zeros((num_best_test_runs,num_objects,max_steps,num_categories))
    joint_probs_handcoded = np.zeros((num_best_test_runs,num_objects,max_steps,num_categories))
    learned_steps = []
    handcoded_steps = []
    
    learned_true = []
    handcoded_true = []
    
    avg_prob_learned = np.zeros((num_best_test_runs,num_objects,max_steps))
    avg_prob_handcoded = np.zeros((num_best_test_runs,num_objects,max_steps))
    
    for test_run in range(num_best_test_runs):
        print "RUNNING EP", test_run
        
        # perform one episode with trained policy
        agent.newEpisode()
        task.reset()
        task.env.verbose = False
        rewards = []
        step_counter = 0
        steps = []
        
        print 'executing learned policy...'
        
        while not task.isFinished():
            actionIdx = drawGibbs(agent.learner.wrappingEvaluable.activate(task.getObservation()), temperature=0)
            task.performAction(actionIdx)
            rewards.append(task.getReward())
            
            for cur_loc in range(len(task.objects)):
                joint_probs_learned[test_run,cur_loc,step_counter] = task.objects[cur_loc].joint_prob
                avg_prob_learned[test_run,cur_loc,step_counter] = (np.argmax(task.objects[cur_loc].joint_prob) == task.env.objects[cur_loc])
                #print 'l', test_run, cur_loc, step_counter, action_names[actionIdx], object_names[task.env.objects[cur_loc]], avg_prob_learned[test_run,cur_loc,step_counter], avg_prob_learned[test_run,cur_loc]
                #print 'l', test_run, cur_loc, step_counter, action_names[actionIdx], object_names[task.env.objects[cur_loc]], joint_probs_learned[test_run,cur_loc]
                
            steps.append(action_names[actionIdx])
            step_counter += 1
            
        learned_true.append(task.env.objects)
        learned_steps.append(steps)
        Ep_rewards.append(rewards)
        
        print 'done\n\n'
        
        # perform one episode with hand-coded policy
        agent.newEpisode()
        task.reset()
        task.env.verbose = False
        rewardsHand = []
        step_counter = 0
        steps = []
        
        actionIdx = 0
        
        while not task.isFinished():
            task.performAction(actionIdx)
            rewardsHand.append(task.getReward())
            
            for cur_loc in range(len(task.objects)):
                joint_probs_handcoded[test_run,cur_loc,step_counter] = task.objects[cur_loc].joint_prob
                avg_prob_handcoded[test_run,cur_loc,step_counter] = (np.argmax(task.objects[cur_loc].joint_prob) == task.env.objects[cur_loc])
                #print 'h', test_run, cur_loc, step_counter, action_names[actionIdx], object_names[task.env.objects[cur_loc]], avg_prob_handcoded[test_run,cur_loc,step_counter], avg_prob_handcoded[test_run,cur_loc]
                #print 'h', test_run, cur_loc, step_counter, action_names[actionIdx], object_names[task.env.objects[cur_loc]], joint_probs_handcoded[test_run,cur_loc]
                
            steps.append(action_names[actionIdx])
            actionIdx += 1
            if actionIdx > len(action_names) - 2: actionIdx = 0
            
            step_counter += 1
            
        handcoded_true.append(task.env.objects)
        handcoded_steps.append(steps)
        Ep_rewardsHand.append(rewardsHand)
        
    probCorrect = avg_prob_learned
    probCorrectHand = avg_prob_handcoded
    #print probCorrect
    #print probCorrectHand
    
    #print joint_probs_learned
    #print joint_probs_handcoded
    
    ###################### save rewards per step with the best agent
    outfile = path+"RewardsPerStep-trained.pkl"
    f = open(outfile, 'w')
    pickle.dump(Ep_rewards,f)
    pickle.dump(Ep_rewardsHand, f)
    f.close()
    
    ###################### save accuracy per step with the best agent and hand-coded policy
    outfile = path+"AccuracyPerStep.pkl"
    f = open(outfile, 'w')
    pickle.dump(probCorrect,f)
    pickle.dump(probCorrectHand,f)
    f.close()
    
    ###################### save joint probs per step with the best agent
    outfile = path+"JointProbsPerStep-learned.pkl"
    f = open(outfile, 'w')
    
    pickle.dump(learned_steps, f)
    pickle.dump(joint_probs_learned, f)
    
    pickle.dump(handcoded_steps, f)
    pickle.dump(joint_probs_handcoded, f)
    
    pickle.dump(learned_true, f)
    pickle.dump(handcoded_true, f)
    
    f.close()
    
    ###################### print metadata and plot figures
    grapher = graph(path)
    grapher.print_data()
    grapher.plot_all()
    print 'EVERYTHING IS DONE, should exit now'

