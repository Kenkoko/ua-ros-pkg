__author__ = 'Daniel Ford, dford@email.arizona.edu'

from pybrain.rl.environments import EpisodicTask
from pybrain.utilities import Named, drawIndex, drawGibbs
from scipy import array, clip, linspace, sort, append, exp, ndarray, minimum, maximum, sum, log
from numpy import *
import math

class InfoMaxTask(EpisodicTask, Named):

	def __init__(self, environment, sort_beliefs=True, do_decay_beliefs=True, randomizeWorld=True, uniformInitialBeliefs=True, \
						maxSteps=30, rewardType=1):

		self.verbose = False
		self.listActions = False    

		# initialize lists of objects and actions
		self.objectNames = environment.objectNames
		self.actionNames = environment.actionNames
		self.numObjects = len(self.objectNames)
		self.numActions = len(self.actionNames)

		self.env = environment
		EpisodicTask.__init__(self, self.env)

		self.numCategories = self.env.numCategories

		self.randomizeWorld = randomizeWorld
		self.uniformInitialBeliefs = uniformInitialBeliefs
		self.maxSteps = maxSteps
		self.rewardscale = 1.0 #/self.maxSteps

		# remember what type of reward is requested
		self.rewardType = rewardType

		# probably need to update these
		self.NORM_ENTROPY = 1
		self.AVERAGE_ENTROPY = 2  

		self.inittask()

	def inittask(self):

		self.steps = 0
		self.createObjects()
		self.initbeliefs()
		self.initRBFs()
		self.loc = 0
		self.beliefsAreNew = True

	def createObjects(self):

		# create random world objects
		if self.randomizeWorld:
			if self.verbose:
				print "Resetting to " + repr(self.numObjects) + " random objects"
    
			# Create random objects
			self.objects = []
			for i in range(self.numObjects):
				self.objects.append(worldObject(self.objectNames[i], self.numCategories, self.numActions, self.numObjects))
			
		else:
			
			if self.verbose:
				print "Resetting to " + repr(self.numObjects) + " NOT random objects"		

	# initialize uniform beliefs	
	def inituniformbeliefs(self):

		beliefs = []
		for obj in self.objects:
			beliefs.append(obj.objProbs)
		beliefs = array(beliefs)
		self.beliefs = beliefs		

	# initialize beliefs and entropy
	def initbeliefs(self):

		self.inituniformbeliefs()
		self.maxentropy = self.entropy()
		if not self.uniformInitialBeliefs:		# non-random beliefs won't work at the moment
			self.initnonrandombeliefs()

	# need to update... if this is needed        
	def initnonrandombeliefs(self):

		pass

	# reset task
	def reset(self):

		EpisodicTask.reset(self)
		self.inittask()			
		print "******************************"

	def initRBFs(self):

		# create RBFs to encode time to completion
		self.numRBFs = 6
		self.sigma = self.maxSteps/self.numRBFs
		self.RBFcenters = linspace(0,self.maxSteps,self.numRBFs)	# will space centers in reals, need to cast to ints
		self.RBFs = zeros(self.numRBFs)		

	def updateRBFs(self):
	
		for rbf in range(self.numRBFs):
			self.RBFs[rbf] = exp(-((self.steps-int(self.RBFcenters[rbf]))**2)/self.sigma)
	
	# returns current belief vector
	def getObservation(self):
		
		"""	
		# sort object PDFs so the data for the object at our current location is first
		currLoc = self.beliefs[self.loc:self.loc+1].copy()		# extract conditional PDFs for current object
		otherLocPre = self.beliefs[0:self.loc].copy()			# slice out PDFs before current object
		otherLocPost = self.beliefs[self.loc+1:].copy()			# slice out PDFs after current object
		otherLoc = concatenate((otherLocPre,otherLocPost))		# rebuild belief vector	
		beliefs = concatenate((currLoc,otherLoc))				
		"""

		# update RBF activations
		self.updateRBFs()

		# flatten list of conditional PDFs for input into the network
		self.beliefs = self.beliefs.flatten()

		# otherwise, chop off old activations
		if self.beliefsAreNew == False:
			self.beliefs = self.beliefs[:self.numCategories*self.numActions]
		# if this is the first step in the episode, just set that flag
		elif self.beliefsAreNew:
			self.beliefsAreNew = False

		# append RBF activations to self.beliefs		
		self.beliefs = concatenate((self.beliefs,self.RBFs)) 

		return self.beliefs
	
	# send chosen task to environment to be performed
	def performAction(self, action):

		if type(action) == ndarray:
			# Take the max, or random among them if there are several equal maxima
			action = drawGibbs(action, temperature=0)

		self.steps += 1

		""" Important to note that we immediately update beliefs after performing action
		 (maybe not needed but it would make credit-assignment harder) """
		#self.loc = self.env.performAction(action)

		# Get sensors and do belief updates before calling reward
		sensors = self.env.getSensors(action)
		self.update_beliefs(sensors, action)

		# Below two lines were cut and pasted from EpisodicTask.performAction()
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

		if self.rewardType == self.AVERAGE_ENTROPY:
			reward = self.entropy()/self.numObjects
		else:
			reward = (self.maxentropy - self.entropy())/self.maxentropy

		#return -reward		# needs to be negative to allow PGPE to minimize as designed
		return reward

	# update beliefs on each object, then compose individual beliefs into the full array
	def update_beliefs(self, sensors, action):

		# send PDF from sensor to update object PDF
		self.objects[sensors.location].updateProbs(action, sensors.beliefs)

		# construct belief list from individual object PDFs
		beliefs = []
		for obj in self.objects:
			beliefs.append(obj.objProbs)
		beliefs = array(beliefs)

		self.beliefs = beliefs

	# calculate entropy of beliefs				
	def entropy(self):
		ent=0;
		for objbelief in self.beliefs:
			ent -= (log(objbelief) * objbelief).sum()
		return ent

	@property
	def indim(self):
		return len(self.actionNames)		# number of actions we can take (minus reset)
    
	@property
	def outdim(self):
		return len(self.getObservation()) 	# belief vector

#####################################################################################
# class definitions for objects in the world and object properties that can be sensed     
class worldObject(Named):    

	# initialize object class
	def __init__(self, name, numCategories, numActions, numObjects):

		self.objName = name					# this object's name
		self.numCategories = numCategories	# number of object categories
		self.numObjects = numObjects		# number of objects
		self.numActions = numActions		# number of possible actions
		self.actionCount = zeros(self.numActions)
		self._initUniformBeliefs()			# PDF over object names
	
	# initialize probabilities to be uniform
	def _initUniformBeliefs(self):

		# initialize conditional probabilities to uniform 
		self.objProbs = tile(1./self.numCategories, (self.numActions,self.numCategories))

		# update joint probabilities
		self._updateJointProbs()

	# update conditional probabilities and joint probability
	# takes the type of the last sensing action performed (integer) and the element of the returned PDF corresponding to that action type (float) 
	def updateProbs(self, actionType, currentPDF):

		"""
		# if we sensed instead of moved or reset, also update beliefs
		if actionType < 4:
		"""

		#print "action ", actionType

		self.objProbs[actionType] *= self.actionCount[actionType]
		self.objProbs[actionType] += currentPDF
		self.objProbs[actionType] /= self.actionCount[actionType]+1

		# increment action list for that type of action		
		self.actionCount[actionType] += 1

		# update joint probabilities
		self._updateJointProbs()

	# update joint probabilities
	def _updateJointProbs(self):

		# create joint probabilities from conditional probs
		self.jointProb = ones((self.numCategories))
		for condprob in self.objProbs:	
			self.jointProb *= condprob

		# normalize
		s = sum(self.jointProb)
		self.jointProb /= s

	# will need to reset all object probabilities
	def reset(self):
		
		self._initUniformBeliefs()
		self.actionCount = zeros(self.numActions)						

if __name__ == "__main__":

	# just some code for testing...
	PDF1 = [0.9, 0.05, 0.05]
	PDF2 = [0.12, 0.71, 0.17]

	cats = 3
	actions = 4
	objs = 1

	actionType = 3

	testObject0 = worldObject("Obj 1", cats, actions, objs)

	print "Obj 0"
	print testObject0.objProbs

	testObject0.updateProbs(actionType,PDF1)
	print testObject0.actionCount[actionType]

	print "Obj 0"
	print testObject0.objProbs

	actionType = 1

	testObject0.updateProbs(actionType,PDF2)
	print testObject0.actionCount[actionType]

	print "Obj 0"
	print testObject0.objProbs
	print testObject0.jointProb
