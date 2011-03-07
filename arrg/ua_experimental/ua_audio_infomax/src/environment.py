#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_infomax')
import rospy

__author__ = 'Daniel Ford, dford@email.arizona.edu'

from random import random, choice
from scipy import zeros
from numpy import *
from math import *
from std_msgs.msg import String

from pybrain.utilities import Named, drawIndex
from pybrain.rl.environments.environment import Environment

# import ROS services
from ua_audio_infomax.srv import *

class InfoMaxEnv(Environment, Named):

	# current manipulations are pick up, drop, push, squeeze
	def __init__(self, randomizeWorld=True, numObjects=5, probs=None):

		self.verbose = False
		self.listActions = False
		self.randomizeWorld = randomizeWorld        

		# initialize lists of objects and actions
		#self.objectNames = ["Obj 0", "Obj 1", "Obj 2", "Obj 3", "Obj 4", "Obj 5", "Obj 6"]
		#self.objectNames = ["Obj 0", "Obj 1", "Obj 2", "Obj 3", "Obj 4"]
		#self.objectNames = ["Obj 0", "Obj 1", "Obj 2"]
		self.objectNames = ["Obj 0", "Obj 1", "Obj 2", "Obj 3"]
		
		self.numObjects = len(self.objectNames)
		#self.actionNames = [String(s) for s in ("pick up", "drop", "push", "squeeze", "move left", "move right", "reset")]
		self.actionNames = ["pick up", "drop", "push", "squeeze", "move left", "move right", "reset"]
		self.numActions = len(self.actionNames)

		# set up "request action" client here
		rospy.init_node('infomaxAgent')

		self.reset()

	# reset environment variables
	def reset(self):

		if self.randomizeWorld:
			if self.verbose:
				print "Resetting to " + repr(self.numObjects) + " random objects"
    
			# Create random objects
			self.objects = []
			for i in range(self.numObjects):
				self.objects.append(worldObject(self.objectNames[i], self.numActions-3, self.numObjects))
			
		else:
			pass
			"""
			if self.verbose:
				print "Resetting to " + repr(self.numObjects) + " NOT random objects"

			self.objects = [ # define objects here ]
			self.objects = self.objects[0:self.numObjects]  # trim the array down to the true size of the objects.
			"""

		# list to keep track of the actions performed
		self.action_list = []

	# get observation from environment
	def getSensors(self, action):

		# send "sense" service request 
		rospy.wait_for_service('InfoMax')	
		try:
			# create service definition
			sense = rospy.ServiceProxy('InfoMax',InfoMax)
			# call service, which returns PDF corresponding to action we requested
			obs = sense(self.objectNames, self.actionNames, action)	

			# return only the PDF from the robot
			return obs	

		# if service fails, exit
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	# perform action selected by the network
	def performAction(self, action):

		# send "move/manipulate" service request
		rospy.wait_for_service('InfoMax') 	
		try:
			move = rospy.ServiceProxy('InfoMax',InfoMax)	
			# call service and only grab new location
			response = move(self.objectNames, self.actionNames, action)	

			# return only the robot's location
			return response.location

		# service failure handler
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


	"""
		# would need to be updated to work
		if self.verbose:
			print self.sensor2str(self.lastsensor)
		if self.listActions:
			self.action_list.append(self.actions[action])

	# would need to be updated to work
	def sensor2str(self, sensor):
		retval = "Location: " + str(sensor.loc);
		if sensor.manip is not None:
			obj = self.objects[sensor.loc]
			prop = obj.properties[ sensor.manip ]
			retval +=  ", Sampled Property: " + prop.property_name
			retval +=  ", Value: " + prop.values[sensor.val]
		return retval
	"""   

# class definitions for objects in the world and object properties that can be sensed     
class worldObject(Named):    

	# initialize object class
	def __init__(self, name, actions, numObjects):

		self.objName = name					# this object's name
		self.numObjects = numObjects		# number of objects
		self.actions = actions				# number of possible actions
		self.numActions = []				# init empty list
		for i in range(0,actions):			# init number of actions performed to zero for each action type
			self.numActions.append(0)

		self._initUniformBeliefs()			# PDF over object names
	
	# initialize probabilities to be uniform
	def _initUniformBeliefs(self):

		self.objProbs = []					# conditional probabilities
		self.jointProb = []					# joint probability

		# initialize conditional probabilities
		for i in range(0,self.actions):					# for each action type
			actionProb = []								# create one PDF over object names
			for i in range(0,self.numObjects):
				actionProb.append(1./self.numObjects)
			self.objProbs.append(actionProb)			# append it to create the full objProbs list

		# update joint probabilities
		self._updateJointProbs()

	# update conditional probabilities and joint probability
	# takes the type of the last sensing action performed (integer) and the element of the returned PDF corresponding to that action type (float) 
	def updateProbs(self, actionType, currentPDF):

		# update conditional probabilities
		for i in range(self.numObjects):
			self.objProbs[actionType][i] *= self.numActions[actionType]
			self.objProbs[actionType][i] += currentPDF[i]
			self.objProbs[actionType][i] *= 1./(self.numActions[actionType]+1)

		# increment action list for that type of action		
		self.numActions[actionType] += 1

		# update joint probabilities
		self._updateJointProbs()

	# update joint probabilities
	def _updateJointProbs(self):
		self.jointProb = []  # But use NumPy np.ones(self.numCategories)sum(self.jointProb)
		# initialize joint probabilities
		for i in range(self.numObjects):
			prod = 1.
			for j in range(self.actions):
				prod *= self.objProbs[j][i]
			self.jointProb.append(prod)
		# normalize
		s = sum(self.jointProb)
		for i in range(len(self.jointProb)):
			self.jointProb[i] /= s
			
	
	# will need to reset all object probabilities
	def reset(self):
		
		self._initUniformBeliefs()

		for i in range(0,len(actions)):			# init number of actions performed to zero for each action type
			self.numActions.append(0)								


