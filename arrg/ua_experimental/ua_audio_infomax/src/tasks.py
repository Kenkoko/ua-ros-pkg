__author__ = 'Daniel Ford, dford@email.arizona.edu'

from pybrain.rl.environments import EpisodicTask
from pybrain.utilities import Named, drawIndex, drawGibbs
from scipy import array, clip, linspace, sort, append, exp, ndarray, minimum, maximum, sum, log
from numpy import *
import math

class InfoMaxTask(EpisodicTask, Named):

	def __init__(self, environment, do_rbf=True, sort_beliefs=True, do_decay_beliefs=True, \
					randomizeWorld=True, uniformInitialBeliefs=True, maxSteps=30, numObjects=5, \
					rewardType=1):

		self.env = environment
		EpisodicTask.__init__(self, self.env)

		self.randomizeWorld = randomizeWorld
		self.uniformInitialBeliefs = uniformInitialBeliefs
		self.maxSteps = maxSteps
		self.rewardscale = 1.0 #/self.maxSteps
		# remember what type of reward is requested
		self.rewardType = rewardType
		self.inittask()

		# probably need to update these
		self.NORM_ENTROPY = 1
		self.AVERAGE_ENTROPY = 2  

		# need to figure out whether these are needed
		#self.sort_beliefs = sort_beliefs
		#self.do_decay_beliefs = do_decay_beliefs

	def inittask(self):
		self.t = 0
		self.steps = 0
		self.initbeliefs()
		self.loc = 0	

	# initialize uniform beliefs	
	def inituniformbeliefs(self):

		beliefs = []
		for i in range(self.env.numObjects):
			beliefs.append(self.env.objects[i].jointProb)	# assume that jointProbs are currently uniform
			#print self.env.objects[i].objProbs				# (will be the case upon initialization)

		beliefs = array(beliefs)
		self.beliefs = beliefs
		
	# initialize beliefs and entropy
	def initbeliefs(self):
		self.inituniformbeliefs()
		self.maxentropy = self.entropy()
		if not self.uniformInitialBeliefs:		# non-random beliefs won't work at the moment
			self.initnonrandombeliefs()

	# need to update...        
	def initnonrandombeliefs(self):

		pass

	# reset task
	def reset(self):
		reset = 6
		self.stepCount = 0
		EpisodicTask.reset(self)
		self.inittask()
		self.env.performAction(reset)			

	# returns current belief vector
	def getObservation(self):
	
		# flatten list of PDFs for input into the network
		self.beliefs = self.beliefs.flatten()
		return self.beliefs
	
	# send chosen task to environment to be performed
	def performAction(self, action):

		if type(action) == ndarray:
			# Take the max, or random among them if there are several equal maxima
			action = drawGibbs(action, temperature=0)
		self.steps += 1

		""" Important to note that we immediately update beliefs after performing action
		 (maybe not needed but it would make credit-assignment harder) """
		self.env.performAction(action)

		# Get sensors and do belief updates before calling reward
		sensors = self.env.getSensors(action)
		self.update_beliefs(sensors, action)

		# Below two lines were cut and paste from EpisodicTask.performAction()
		self.addReward()
		self.samples += 1

	# set task completion criteria
	def isFinished(self):

		# max info reached, max number of actions performed, or failure condition
		if self.steps >= self.maxSteps:
			return True
		else:
			return False

	# compute and return the current reward (i.e. corresponding to the last action performed)
	def getReward(self):

		# possible values are bounded between maxentropy and 0.
		# We can try to minimize the 
		#return self.maxentropy - self.entropy()
		if self.rewardType == self.AVERAGE_ENTROPY:
			reward = self.entropy()/self.env.numObjects
		else:
			reward = (self.maxentropy - self.entropy())/self.maxentropy
		# return norm_reward
		return reward; #*norm_reward

	"""
	# needs to be updated
	def decay_beliefs(self):
		self.beliefs = self.beliefs*0.95 + (1.0/self.numObjects)*0.05
		for loc in range(len(self.beliefs)):
			for prop in range(len(self.beliefs[loc])):
				self.beliefs[loc][prop] = self.beliefs[loc][prop]\
										/ self.beliefs[loc][prop].sum()
	"""

	#def update_beliefs(self, sensors=robot_sensor()):
	def update_beliefs(self, sensors, action):

		# decay the beliefs every time step, to keep the agent moving
		#if self.do_decay_beliefs:
		#	self.decay_beliefs()

		# update beliefs if we performed a sensing action (as opposed to a move or reset)
		if action < 4:
			# send PDF from sensor to update object PDF
			self.env.objects[sensors.location].updateProbs(action, sensors.beliefs)

		# construct belief list from individual object PDFs
		k = 0
		beliefs = zeros(self.env.numObjects**2)
		for i in range(self.env.numObjects):
			for j in range(self.env.numObjects):
				beliefs[k] = self.env.objects[i].jointProb[j]
				k += 1
		self.beliefs = beliefs

	# calculate entropy of beliefs				
	def entropy(self):
		ent=0;
		for objbelief in self.beliefs:
			ent -= (log(objbelief) * objbelief).sum()
		return ent

	# print beliefs showing one object PDF per line 
	def showBeliefs(self):
	
		for idx in range(self.env.numObjects):
			for i in range(self.env.numObjects):
				print self.beliefs[i+idx*self.env.numObjects],
			print "\r"

	@property
	def indim(self):
		return len(self.env.actionNames)-1	# number of actions we can take (minus reset)
    
	@property
	def outdim(self):
		return len(self.getObservation()) 	# belief vector
