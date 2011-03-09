__author__ = 'Daniel Ford, dford@email.arizona.edu'

# import utility functions
import datetime, pickle, sys
from scipy import *
from scipy import array
from numpy import *
from numpy.random import randn

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

class InfoMaxAgent():

	def __init__(self, option):

		# examine command-line argument for learning algorithm
		if option[1] == "PGPE":
			self._PGPE = True; self._CMAES = False	
		elif option[1] == "CMAES":
			self._PGPE = False; self._CMAES = True
		else: self._PGPE = True

		# objects and their categories
		self.objDict = {	"Obj 0":"1",
							"Obj 1":"2",
							"Obj 2":"2",
							"Obj 3":"1",
							"Obj 4":"3",
							"Obj 5":"1",
							"Obj 6":"1",
							"Obj 7":"3"		}		

		self.numCategories = 3

		# possible actions
		self.actionDict = {"pick up":0, "drop":1, "push":2, "squeeze":3, "move left":4, "move right":5, "reset":6}
		
		#self.catNames = ["Cat 1", "Cat 2", "Cat 3", "Cat 4"]
		#self.objectNames = ["Obj 0", "Obj 1", "Obj 2", "Obj 3", "Obj 4", "Obj 5", "Obj 6"]
		#self.objectNames = ["Obj 0", "Obj 1", "Obj 2", "Obj 3", "Obj 4"]
		#self.objectNames = ["Obj 0", "Obj 1", "Obj 2", "Obj 3"]
		#self.objectNames = ["Obj 0", "Obj 1", "Obj 2"]
		#self.objectNames = ["Obj 0","Obj 1"]
		#self.actionNames = ["pick up", "drop", "push", "squeeze", "move left", "move right", "reset"]

	# run trained network in our environment
	def run(self, maxSteps):

		#self.env = InfoMaxEnv(self.catNames, self.objectNames, self.actionNames)
		self.env = InfoMaxEnv(self.objDict, self.actionDict, self.numCategories)
		self.task = InfoMaxTask(self.env, maxSteps=maxSteps, \
					do_decay_beliefs = True, uniformInitialBeliefs = True)
		self.task.reset()

		# load network if we're just running, not training
		self.params = pickle.load(open('infomaxNet.pkl'))
		self.params.sorted = False
		self.params.sortModules()

		print "\n"
		while not self.task.isFinished():
	
			# get initial observation of environment	
			obs_pre = self.task.getObservation()

			print "State pre"
			#print self.task.showBeliefs()		# use formatted print beliefs function
			print self.task.getObservation()

			# send observation to net for an action vector
			action = self.params.activate(obs_pre)	

			# send action vector to robot
			self.task.performAction(action) 

			print "State post"
			#print self.task.showBeliefs()
			print self.task.getObservation()

			# calculate and show reward
			print "reward",self.task.getReward()
			print "\n"

		print "total reward =",self.task.getTotalReward()
		print "\n" 

	# train a new network with PGPE or CMAES
	def train(self, episodes, maxSteps):
 	
		avgReward = 0
		numHidden = 16

		# set up environment and task
		#self.env = InfoMaxEnv(self.catNames, self.objectNames, self.actionNames)
		self.env = InfoMaxEnv(self.objDict, self.actionDict, self.numCategories)
		self.task = InfoMaxTask(self.env, maxSteps=maxSteps, \
					do_decay_beliefs = True, uniformInitialBeliefs = True)

		# create neural net and learning agent
		self.params = buildNetwork(self.task.outdim, numHidden, self.task.indim, \
						bias=True, outclass=SoftmaxLayer)
		if self._PGPE:
			self.agent = OptimizationAgent(self.params, PGPE(minimize=True,verbose=False))
		elif self._CMAES:
			self.agent = OptimizationAgent(self.params, CMAES(minimize=True,verbose=False))

		# init and perform experiment
		exp = EpisodicExperiment(self.task, self.agent)

		for i in range(episodes):        
			exp.doEpisodes(1)
			avgReward += self.task.getTotalReward()
			print "reward episode ",i,self.task.getTotalReward()

		# print initial info
		print "\naverage reward over training = ",avgReward/episodes

		# save trained network
		self._saveWeights()

	# save and pickle trained weights for later reuse
	def _saveWeights(self):

		# import weights into network and save network

		if self._PGPE:
			for i in range(len(self.params.params)):
				self.params.params[i] = self.agent.learner.current[i]
			pickle.dump(self.params, open('infomaxNet.pkl','w'))

		elif self._CMAES:

			################ following code came from WWInfoMaxCMAES.py script from ICDL 2010 paper
			arz = randn(self.agent.learner.numParameters, self.agent.learner.batchSize)
			arx = tile(self.agent.learner.center.reshape(self.agent.learner.numParameters, 1),\
			(1, self.agent.learner.batchSize)) + \
			self.agent.learner.stepSize * dot(dot(self.agent.learner.B, self.agent.learner.D), arz)
			# Go through the parameters and pick the current best 
			arfitness = zeros(self.agent.learner.batchSize)
			for k in xrange(self.agent.learner.batchSize):
			  self.agent.learner.wrappingEvaluable._setParameters(arx[:, k]);
			  arfitness[k] = self.agent.learner._BlackBoxOptimizer__evaluator\
			(self.agent.learner.wrappingEvaluable)

			# Sort by fitness and compute weighted mean into center
			tmp = sorted(map(lambda (x, y): (y, x), enumerate(ravel(arfitness))))
			arfitness = array(map(lambda x: x[0], tmp))
			arindex = array(map(lambda x: int(x[1]), tmp))

			arz = arz[:, arindex]
			curparams = arx[:, arindex[0]];

			# update network weights with selected parameters
			for i in range(len(self.params.params)):
				self.params.params[i] = curparams[i]
			# save trained network
			pickle.dump(self.params, open('infomaxNet.pkl','w'))

