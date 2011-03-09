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
		self.catNames = environment.catNames
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

		# need to figure out whether these are needed
		#self.sort_beliefs = sort_beliefs
		self.do_decay_beliefs = do_decay_beliefs  

		self.inittask()

	def inittask(self):

		self.steps = 0
		self.createObjects()
		self.initbeliefs()
		self.loc = self.env.performAction(self.env.actionDict["reset"])

	def createObjects(self):

		# create random world objects
		if self.randomizeWorld:
			if self.verbose:
				print "Resetting to " + repr(self.numObjects) + " random objects"
    
			# Create random objects
			self.objects = []
			for i in range(self.numObjects):
				self.objects.append(worldObject(self.objectNames[i], self.numCategories, self.numActions-3, self.numObjects))
			
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

	# need to update...        
	def initnonrandombeliefs(self):

		pass

	# reset task
	def reset(self):

		EpisodicTask.reset(self)
		self.inittask()			
		print "************************************"

	# returns current belief vector
	def getObservation(self):
	
		# sort object PDFs so the data for the object at our current location is first
		currLoc = self.beliefs[self.loc:self.loc+1].copy()		# extract conditional PDFs for current object
		otherLocPre = self.beliefs[0:self.loc].copy()			# slice out PDFs before current object
		otherLocPost = self.beliefs[self.loc+1:].copy()			# slice out PDFs after current object
		otherLoc = concatenate((otherLocPre,otherLocPost))		# rebuild belief vector	
		beliefs = concatenate((currLoc,otherLoc))				

		# flatten list of PDFs for input into the network
		self.beliefs = beliefs.flatten()
		return self.beliefs
	
	# send chosen task to environment to be performed
	def performAction(self, action):

		if type(action) == ndarray:
			# Take the max, or random among them if there are several equal maxima
			action = drawGibbs(action, temperature=0)
		self.steps += 1

		""" Important to note that we immediately update beliefs after performing action
		 (maybe not needed but it would make credit-assignment harder) """
		self.loc = self.env.performAction(action)

		# Get sensors and do belief updates before calling reward
		sensors = self.env.getSensors(action, self.loc)

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
			reward = self.entropy()/self.numObjects
		else:
			reward = (self.maxentropy - self.entropy())/self.maxentropy
		# return norm_reward
		return reward; #*norm_reward

	"""
	# needs to be updated
	def decay_beliefs(self):

		self.beliefs *= 0.95 + (1.0/self.numObjects)*0.05
		for loc in range(len(self.beliefs)):
			for prop in range(len(self.beliefs[loc])):
				self.beliefs[loc][prop] = self.beliefs[loc][prop]\
										/ self.beliefs[loc][prop].sum()
	"""

	#def update_beliefs(self, sensors=robot_sensor()):
	def update_beliefs(self, sensors, action):

		# send PDF from sensor to update object PDF
		self.objects[sensors.location].updateProbs(action, sensors.beliefs)

		# construct belief list from individual object PDFs
		beliefs = []
		for obj in self.objects:
			beliefs.append(obj.objProbs)
		beliefs = array(beliefs)

		self.beliefs = beliefs

		#if self.do_decay_beliefs: self.decay_beliefs()

	# calculate entropy of beliefs				
	def entropy(self):
		ent=0;
		for objbelief in self.beliefs:
			ent -= (log(objbelief) * objbelief).sum()
		return ent

	"""
	# print beliefs showing one object PDF per line 
	def showBeliefs(self):
	
		for idx in range(self.numObjects):
			for i in range(self.numObjects):
				print self.beliefs[i+idx*self.numObjects],
			print "\r"
	"""

	@property
	def indim(self):
		return len(self.actionNames)-1	# number of actions we can take (minus reset)
    
	@property
	def outdim(self):
		return len(self.getObservation()) 	# belief vector

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

		# if we sensed instead of moved or reset, also update beliefs
		if actionType < 4:

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

	"""
	# decay beliefs
	def decayBeliefs(self):

		self.objProbs *= 0.95 + (1.0/self.numObjects)*0.05
	"""		

	# will need to reset all object probabilities
	def reset(self):
		
		self._initUniformBeliefs()
		self.actionCount = zeros(self.numActions)						

if __name__ == "__main__":

	# just some code for testing...
	PDF1 = [0.9, 0.05, 0.05]
	PDF2 = [0.6, 0.13, 0.27]

	cats = 3
	actions = 4
	objs = 4

	actionType = 3

	testObject0 = worldObject("Obj 1", cats, actions, objs)
	testObject1 = worldObject("Obj 2", cats, actions, objs)

	print "Obj 0"
	print testObject0.objProbs
	print "Obj 1"
	print testObject1.objProbs

	testObject0.updateProbs(actionType,PDF1)
	print testObject0.actionCount[actionType]

	print "Obj 0"
	print testObject0.objProbs
	print "Obj 1"
	print testObject1.objProbs

	testObject0.updateProbs(actionType,PDF2)
	print testObject0.actionCount[actionType]

	print "Obj 0"
	print testObject0.objProbs
	print "Obj 1"
	print testObject1.objProbs
