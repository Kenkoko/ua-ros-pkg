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
from matplotlib.pyplot import show, figure, xlabel, ylabel, errorbar, legend
from scipy import mean, size, std
from numpy import arange, multiply, zeros, shape
from PlotInfoMaxExample import *

if __name__ == '__main__':

	try:

		# objects and their categories
		numCategories = 6

		# tuples are name, category
		objectNames = [	("Obj 0",2)	]
		actionNames = ["grasp", "lift", "drop", "shake/roll", "place"]

		ets = []

		numbExp = 10 			# number of learning experiments, was 16
		prnts = 10	 			# number of batches per experiment, was 100

		batch = 50  					# number of learning episodes per batch, was 50
		numTestingEps = 15; 			# number of testing episodes per batch, was 8
		maxSteps = 10						# number of steps per episode
		
		numTestRunEps= 10;		# number of episodes to test the best overall agent, was 50

		lrn_rewards = []
		best_params = []
		totalJointProbs = []

		best_reward = -1000;
		for runs in range(numbExp):

			print "\n///////////// STARTING EXPERIMENT",runs

			# set up environment, task, neural net, agent, and experiment
			env = InfoMaxEnv(objectNames, actionNames, numCategories)
			task = InfoMaxTask(env, maxSteps=maxSteps, do_decay_beliefs=True, uniformInitialBeliefs=True)
			net = buildNetwork(task.outdim, task.indim, bias =True, outclass=SoftmaxLayer)
			agent = OptimizationAgent(net, PGPE(storeAllEvaluations=True,minimize=False,verbose=False))
			experiment = EpisodicExperiment(task, agent)

			agent_rewards = []
			ep_rewards = []
			# Learn in batches
			for i in range(prnts):

				print "\n\t@@@@@@@@@@@@@@ STARTING LEARNING BATCH",i

				########################## learn policy
 
				# Have the agent learn
				task.env.listActions = True
				jointProbsList = []
				for bat in range(batch):
					print "\t\tLEARNING EP",bat
					experiment.doEpisodes(1)								# do one episode
					print "\t\t\treward learning episode",bat,task.getReward()	# get reward for this episode
					#print task.objects[0].jointProb
					jointProbsList.append(task.objects[0].jointProb)

				totalJointProbs.append(jointProbsList)						# save joint probs from learning

				########################## test learned policy

				# When a batch is done, evaluate so we can see progress and show learning curves
				curparams = agent.learner.current;
				agent.learner.wrappingEvaluable._setParameters(curparams);

				# Evaluate the current learned policy for numTestingEps episodes
				rewards = []
				#testingJointProbs = []
				for dummy in range(numTestingEps):
					print "\t\tTESTING EP",dummy
					agent.newEpisode()
					# Execute the agent in the environment without learning for one episode.
					# This uses the current set of parameters
					r = agent.learner._BlackBoxOptimizer__evaluator(agent.learner.wrappingEvaluable)
					print "\t\t\treward testing episode",dummy,r
					rewards.append(r)
					#testingJointProbs.append(task.objects[0].jointProb)

				# average of all rewards earned by the current policy running numTestingEps episodes
				total_rewards = mean(a=rewards);

				print "\n\taverage testing reward batch",i,total_rewards
				print "\n"

				# list of average rewards per batch 
				agent_rewards.append(total_rewards)
				# list of reward per episode in this batch
				ep_rewards.append(rewards)
				
				# compare the average reward for this evaluation of the learned policy. If it is better on average than
				# a previous one, then save off the parameters that make up the neural network so we can use it to 
				# perform a single episode
				if total_rewards > best_reward:
					best_reward = total_rewards;
					bestparams = agent.learner.current.copy();
				  
			# list of list of average rewards per batch
			#lrn_rewards.append(agent_rewards)

			# list of lists of rewards per episode in each batch
			lrn_rewards.append(ep_rewards)

			# saving parameters of the best policy
			best_params2 = bestparams.copy();
			# All Batches done.  

		# reshape reward array to plot total number of episodes (batches * testing episodes)
		mult = prnts * numTestingEps
		lrn_rewards = array(lrn_rewards).reshape(numbExp,mult)

		# save and plot rewards per batch during learning
		outfile = "./data/RewardsPerEpisode-learning.pkl"
		print outfile
		f = open(outfile, 'w')
		pickle.dump(lrn_rewards,f)    
		f.close()

		fig1 = figure()
		filename = "./data/RewardsPerEpisode-learning.pkl"
		line1 = plot_rewards(filename, 'gx-','green')
		xlabel("Episode #")
		ylabel("Reward(-H)")
  
		# Now run the best agent
		Ep_rewards = []
		agent.learner.wrappingEvaluable._setParameters(best_params2)
		for dummy in range(numTestRunEps):
			print "RUNNING EP",dummy
			agent.newEpisode()
			task.reset()
			task.env.verbose = False
			rewards = []
			Ep_jointProbs = []

			# perform one episode
			while not task.isFinished():
				Ep_jointProbs.append(task.objects[0].jointProb)
				task.performAction(agent.learner.wrappingEvaluable.activate(task.getObservation()))
				rewards.append(task.getReward())

			Ep_rewards.append(rewards)

		# save and plot rewards per step with the best agent
		outfile = "./data/RewardsPerStep-trained.pkl"
		f = open(outfile, 'w')
		pickle.dump(Ep_rewards,f)   
		f.close()

		fig1 = figure()
		filename = "./data/RewardsPerStep-trained.pkl"
		#line1 = plot_rewards(filename,'r+-','red')
		line1 = plot_episode(filename,'r+-','red')
		xlabel("Step #")
		ylabel("Reward(-H)")

		# save and plot joint probs per episode
		outfile = "./data/JointProbsPerStep-learned.pkl"
		f = open(outfile, 'w')

		catList = []
		for catidx in range(numCategories):
			cat = []
			for tup in range(size(Ep_jointProbs,axis=0)):
				cat.append(Ep_jointProbs[tup][catidx])
			catList.append(cat)

		pickle.dump(catList,f)   
		f.close()

		fig1 = figure()
		filename = "./data/JointProbsPerStep-learned.pkl"
		x_vals, jointProbs = plot_jointProbs(filename, numCategories)
	
		line = plot(x_vals[0], jointProbs[0], "red")
		line = plot(x_vals[1], jointProbs[1], "green")
		line = plot(x_vals[2], jointProbs[2], "blue")
		line = plot(x_vals[3], jointProbs[3], "yellow")
		line = plot(x_vals[4], jointProbs[4], "black")
		line = plot(x_vals[5], jointProbs[5], "cyan")

		xlabel("Step #")
		ylabel("Category Joint Probabilities")	
	
		"""			
		x_vals = arange(length)			# number of joint prob tuples in the list
		stds = std(placehold)			# std dev of one category across entire list
		figure()
		xlabel('Step #')
		ylabel('Joint Prob: 2')
		errorbar(x_vals, placehold, stds, fmt='-') 
		"""

		show()

	except rospy.ROSInterruptException: pass
