#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_infomax')
import rospy

__author__ = 'Daniel Ford, dford@email.arizona.edu'

from random import random, choice
from scipy import zeros
from numpy import *
from math import *
from std_msgs.msg import String
import ua_audio_infomax.msg as InfomaxAction

from pybrain.utilities import Named, drawIndex
from pybrain.rl.environments.environment import Environment

# import ROS services
from ua_audio_infomax.srv import *

class InfoMaxEnv(Environment, Named):

	def __init__(self, objNames, actionNames, numCategories):

		self.numCategories = numCategories
		self.actionNames = actionNames

		# extract object names and categories from tuple list
		self.objectNames = []
		self.objCats = []
		for obj in range(len(objNames)):
			self.objectNames.append(objNames[obj][0])
			self.objCats.append(objNames[obj][1])

		# set up "request action" client here
		rospy.init_node('infomaxAgent')

	# get observation from environment
	def getSensors(self, action):

		location = 0

		# get category of the object in front of the robot
		catID = self.objCats[location]

		# send "sense" service request 
		rospy.wait_for_service('InfoMax')	
		try:
			# create service definition
			sense = rospy.ServiceProxy('InfoMax',InfoMax)

			InfomaxAction.val = action	# write correct value		
	
			# call service, which returns PDF corresponding to action we requested
			#obs = sense(self.objectNames, self.actionNames, self.numCategories, catID, action)	
			obs = sense(self.objectNames, self.actionNames, self.numCategories, catID, InfomaxAction)

			# return only the PDF from the robot
			return obs	

		# if service fails, exit
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
	"""
	# perform action selected by the network
	def performAction(self, action):

		# send "move/manipulate" service request
		rospy.wait_for_service('InfoMax') 	
		try:
			move = rospy.ServiceProxy('InfoMax',InfoMax)	
			# call service and only grab new location
			response = move(self.objectNames, self.actionNames, self.numCategories, 0, action)	

			# return only the robot's location
			return response.location

		# service failure handler
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e  
	"""

