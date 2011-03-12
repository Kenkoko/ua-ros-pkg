#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_infomax')
import rospy, sys, pickle, copy

############################################################################
#
# wrapper for InfoMax reinforcement learning with PyBrain
#
# runs InfoMax agent in batches and collects data for plotting 	
#
############################################################################

__author__ = 'Daniel Ford, dford@email.arizona.edu'

#from agents import GraphingInfoMaxAgent

# import optimization frameworks and agent class
from pybrain.optimization import *
from pybrain.rl.agents import OptimizationAgent
from pybrain.rl.experiments import EpisodicExperiment

# import neural net stuff
from pybrain.structure.networks.network import Network
from pybrain.tools.shortcuts import buildNetwork
from pybrain.structure.modules import SoftmaxLayer

# import tasks and environments
from tasks import InfoMaxTask
from environment import InfoMaxEnv

# math and plotting imports
from matplotlib.pyplot import show, figure, xlabel, ylabel, errorbar
from scipy import mean, size, std
from numpy import arange, multiply

if __name__ == '__main__':

	try:

		maxSteps = 30

		# objects and their categories
		numCategories = 4

		# tuples are name, category
		objectNames = [	("Obj 0",2)	]
		actionNames = ["pick up", "drop", "push", "squeeze"]

		ets = []

		batch = 25  			# number of samples per learning step, 50
		prnts = 10 			# number of times we perform learning, 100
		numbExp = 10 			# number of experiments, 16
		numTestingEps = 8; 		# in this case there isn't *that* much variance, so 5-10 will do

		lrn_rewards = []
		best_params = []

		best_reward = -1000;
		for runs in range(numbExp):

			# set up environment, task, neural net, agent, and experiment
			env = InfoMaxEnv(objectNames, actionNames, numCategories)
			task = InfoMaxTask(env, maxSteps=maxSteps, do_decay_beliefs=True, uniformInitialBeliefs=True)
			net = buildNetwork(task.outdim, task.indim, bias =True, outclass=SoftmaxLayer)
			#agent = OptimizationAgent(net, PGPE(storeAllEvaluations=True,minimize=True,verbose=False))
			agent = OptimizationAgent(net, PGPE(storeAllEvaluations=True,minimize=False,verbose=False))
			experiment = EpisodicExperiment(task, agent)

			agent_rewards = []

			#Do the experiment
			# Learn in batches
			for i in range(prnts):

				# Have the agent learn for 100 episodes
				task.env.listActions = True
				experiment.doEpisodes(batch)

				#print
				#print('Last 50 evaluations: ' + repr(agent.learner._allEvaluations[-50:-1]))
				#print('Actions after learning ' + repr(task.env.action_list))

				# When a batch is done, evaluate so we can see progress and show learning curves
				curparams = agent.learner.current;
				agent.learner.wrappingEvaluable._setParameters(curparams);

				# Evaluate the current learned policy for numTestingEps episodes
				rewards = []
				for dummy in range(numTestingEps):
					agent.newEpisode()
					# Execute the agent in the environment without learning for one episode.
					# This uses the current set of parameters
					r = agent.learner._BlackBoxOptimizer__evaluator(agent.learner.wrappingEvaluable)
					rewards.append(r)

				#print ('interim rewards: ' + repr(rewards))
				#print('Actions after evaluating ' + repr(task.env.action_list))

				# save the average of all the rewards that the evaluation did in its runs
				total_rewards = mean(a=rewards);
				# put this average reward for this agent onto a list so we can plot it later
				agent_rewards.append(total_rewards)

				#print ('After step ' + repr(i*batch) + ', Mean reward: ' + repr(total_rewards) + ' test# ' + repr(i))
				
				# compare the average reward for this evaluation of the learned policy. If it is better on average than
				# a previous one, then save off the parameters that make up the neural network so we can use it to 
				# perform a single episode
				if total_rewards > best_reward:
					best_reward = total_rewards;
					bestparams = agent.learner.current.copy();
				  
			# put the rewards from the last agent onto a total list
			lrn_rewards.append(agent_rewards)
			best_params2 = bestparams.copy();
			# All Batches done.  

		# Now save off the contents of the lrn_rewards lists so we can plot it later
		outfile = "PGPE-learning.pkl"
		print outfile
		f = open(outfile, 'w')
		pickle.dump(lrn_rewards,f)    
		f.close()

		# display rewards per episode
		x_vals = multiply(arange(size(lrn_rewards, axis=1)),batch)
		stds = std(lrn_rewards,axis=0)
		figure()
		xlabel('Episode #')
		ylabel('-H')
		line1 = errorbar(x_vals, mean(lrn_rewards,axis=0), stds, fmt='-')
		#legend((line1[0]),('Learning'), 'upper left') 		# looks like a legend 
		  
		# Now do 50 Episodes with the best agent
		numEps= 50;
		Ep_rewards = []
		#net._setParameters(bestparams);
		agent.learner.wrappingEvaluable._setParameters(best_params2)
		for dummy in range(numEps):
			agent.newEpisode()
			task.reset()
			task.env.verbose = False
			#task.env.listActions = True
			rewards = []
			# perform one episode
			while not task.isFinished():
				# task.performAction(best_params[0].activate(task.getObservation()))
				task.performAction(agent.learner.wrappingEvaluable.activate(task.getObservation()))
				rewards.append(task.getReward())
			#print('Total reward: ' + repr(sum(rewards)))

			Ep_rewards.append(rewards)
			#print('Actions ' + repr(task.env.action_list))

		outfile = "PGPE-learned-Ep.pkl"
		f = open(outfile, 'w')
		pickle.dump(Ep_rewards,f)
		#pickle.dump(task.env.action_list,f)    
		f.close()

		# disply average reward per step in an episode
		# To display the average reward as a function of timestep in an episode,
		# we need to average the lists in all_rewards.
		x_vals = arange(size(Ep_rewards, axis=1))
		stds = std(Ep_rewards,axis=0)
		figure()
		xlabel('Step #')
		ylabel('-H')
		errorbar(x_vals, mean(Ep_rewards,axis=0), stds, fmt='-')  
		show()

	except rospy.ROSInterruptException: pass
