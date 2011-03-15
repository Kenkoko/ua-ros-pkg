#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_infomax')
import rospy, sys, pickle, copy, datetime, os

############################################################################
#
# wrapper for InfoMax reinforcement learning with PyBrain
#
# runs InfoMax agent in batches and collects and plots data 	
#
############################################################################

__author__ = 'Daniel Ford, dford@email.arizona.edu'

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

# plotting import
from graphExperiment import *

###################### initialize experiment

# categories and objects 
numCategories = 6
objectNames = [	("Obj 0",3)	]		# tuples are name, category
catID = objectNames[0][1]
actionNames = ["grasp", "lift", "drop", "shake/roll", "place"]

# experiment parameters
numbExp = 2 			# number of learning experiments, was 16
prnts = 100					# number of batches per experiment, was 100
batch = 50	  					# number of learning episodes per batch, was 50
numTestingEps = 10; 			# number of testing episodes per batch, was 8
numTestRunEps = 25;				# number of episodes to run the best overall agent, was 50
maxSteps = 10						# number of steps per episode

# init structures for rewards and network parameters
lrn_rewards = []
best_params = []
totalJointProbs = []
best_reward = -1000;

# save off experiment parameters for each run
def pickle_metadata(filename, timestamp, netparams):

	f = open(filename, 'w')

	pickle.dump(timestamp,f)					# timestamp
	pickle.dump(numCategories,f)				# number of categories
	pickle.dump(objectNames,f)					# object name and category
	pickle.dump(actionNames,f)					# action names

	pickle.dump(numbExp,f)						# number of experiments
	pickle.dump(prnts,f)						# number of learning batches per experiment
	pickle.dump(batch,f)						# number of episodes per batch
	pickle.dump(numTestingEps,f)				# number of testing episodes per batch
	pickle.dump(numTestRunEps,f)				# number of episodes to run the best agent
	pickle.dump(maxSteps,f)						# number of steps per episode 

	pickle.dump(netparams,f)					# parameters of trained policy

	f.close()

if __name__ == '__main__':

	try:

		###################### run [numbExp] experiments, each with [prnts] batches
		###################### each batch has [batch] episodes with [maxSteps] per episode 
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
			agent_rewards.append(0)
			for i in range(prnts):

				print "\n\t@@@@@@@@@@@@@@ STARTING LEARNING BATCH",i

				########################## learn policy
 
				# Have the agent learn
				task.env.listActions = True
				#jointProbsList = []
				for bat in range(batch):
					print "\t\tLEARNING EP",bat
					experiment.doEpisodes(1)								# do one episode
					print "\t\t\treward learning episode",bat,task.getReward()	# get reward for this episode
					#print task.objects[0].jointProb
					#jointProbsList.append(task.objects[0].jointProb)

				#totalJointProbs.append(jointProbsList)						# save joint probs from learning

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
			lrn_rewards.append(agent_rewards)

			# list of lists of rewards per episode in each batch
			#lrn_rewards.append(ep_rewards)

			# saving parameters of the best policy
			best_params2 = bestparams.copy();
			# All Batches done.  

		print "\n///////////// LEARNING DONE"

		###################### set path to save experiment files and metadata
		path = ""
		#path = "data"	
		#os.system("cd .."); os.system("chmod -R 777 data")	# brute-force method to ensure we have the right permissions
		timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
		#path = "./" + path + "/" + timestamp + "/"
		#os.mkdir(path)
		filename = "experiment.desc"
		# save metadata, including trained policy parameters
		pickle_metadata(path+filename, timestamp, best_params2)

		###################### save rewards per batch during learning
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
		for dummy in range(numTestRunEps):
			print "RUNNING EP",dummy

			# perform one episode with trained policy
			agent.newEpisode()
			task.reset()
			task.env.verbose = False
			rewards = []
			Ep_jointProbs = []
			PrCorrect = []
			correctCount = 0

			#Ep_rewards.append(zeros(maxSteps))
			while not task.isFinished():
				Ep_jointProbs.append(task.objects[0].jointProb)
				task.performAction(agent.learner.wrappingEvaluable.activate(task.getObservation()))
				rewards.append(task.getReward())
				if argmax(task.objects[0].jointProb) == catID:
					correctCount += 1
				PrCorrect.append(correctCount/task.steps)

			Ep_rewards.append(rewards)
			probCorrect.append(PrCorrect)

			# perform one episode with hand-coded policy (just cycles through actions in reverse order)
			agent.newEpisode()
			task.reset()
			task.env.verbose = False
			rewardsHand = []
			Ep_jointProbsHand = []
			PrCorrectHand = []
			correctCountHand = 0
			
			#Ep_rewardsHand.append(zeros(maxSteps))
			actionIdx = len(actionNames)-1
			while not task.isFinished():
				Ep_jointProbsHand.append(task.objects[0].jointProb)
				task.performAction(actionIdx)
				rewardsHand.append(task.getReward())
				if argmax(task.objects[0].jointProb) == catID:
					correctCountHand += 1
				PrCorrectHand.append(correctCountHand/task.steps)			
				actionIdx -= 1
				if actionIdx < 0: actionIdx = len(actionNames)-1
			
			Ep_rewardsHand.append(rewardsHand)
			probCorrectHand.append(PrCorrectHand)

		###################### save rewards per step with the best agent
		outfile = path+"RewardsPerStep-trained.pkl"
		f = open(outfile, 'w')
		pickle.dump(Ep_rewards,f)   
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

		# pull out individual probabilities to save
		catList = []
		for catidx in range(numCategories):
			cat = []
			for tup in range(size(Ep_jointProbs,axis=0)):
				cat.append(Ep_jointProbs[tup][catidx])
			catList.append(cat)

		pickle.dump(catList,f)   
		f.close()

		###################### print metadata and plot figures
		grapher = graph(path)
		grapher.print_data()
		grapher.plot_all()

	except rospy.ROSInterruptException: pass
