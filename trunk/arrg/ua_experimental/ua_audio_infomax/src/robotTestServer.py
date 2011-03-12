#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_infomax')

# test node that responds to service requests from the InfoMax agent
# intended to mimic robot controller node

from numpy import *
from math import *

# import services and messages
from ua_audio_infomax.srv import *
import ua_audio_infomax.msg as InfomaxAction

import rospy
from random import *
from PDF import PDF_library, PDF

# init node w/ services, topics, and subscriptions
class robotTestServer():

	def __init__(self):

		self.objectNames = None
		self.actionNames = None
		self.numObjects = None
		self.loc = 0

		rospy.init_node('robotTestServer')		# init node
		# init services
		request = rospy.Service('InfoMax', InfoMax, self._handleSvcRequest)
		print "Ready to run the robot"

		self.PDFsCreated = False

	# just keep node running
	def run(self):
		rospy.spin()

	# reset robot and variables
	def _reset(self):
		self.loc = 0
		self.PDFsCreated = False

	# callback for service request
	def _handleSvcRequest(self,req):

		self.numCategories = req.numCats
		self.objectNames = req.objectNames
		self.numObjects = len(req.objectNames)
		self.actionNames = req.actionNames

		# perform action and get PDF (or None if we moved or reset the robot)
		PDF, location = self._performAction(req)

		# return object names, current location, PDF from sensing
		return InfoMaxResponse(PDF, location) 

	# instruct the robot to move to or sense the selected object
	def _performAction(self,req):

		"""
		# move, reset, or sense object
		if self.actionNames[req.actionID] == "move left":

			# move left in a ring
			self.loc -= 1
			if self.loc < 0: self.loc = len(self.objectNames)-1

			print "\n"
			print "Moved left to ", self.objectNames[self.loc]
			return None, self.loc

		elif self.actionNames[req.actionID] == "move right": 

			# move right in a ring
			self.loc += 1
			if self.loc > (len(self.objectNames)-1): self.loc = 0

			print "\n"
			print "Moved right to ", self.objectNames[self.loc]
			return None, self.loc

		elif self.actionNames[req.actionID] == 'reset':

			print "\n"
			print "************ RESET ************"
			self._reset()
			return None, self.loc

		else:

			print self.actionNames[req.actionID]
			PDF = self._getSensors(req)
			print "\n"
			print "At ", self.objectNames[self.loc]
			print "Performed ", self.actionNames[req.actionID]
			print "Sensed ", PDF
			return PDF, self.loc
		"""

		PDF = self._getSensors(req)
		print "\n"
		print "At ", self.objectNames[self.loc]
		print "Performed ", self.actionNames[req.actionID.val]
		print "Sensed ", PDF
		return PDF, self.loc		

	# simulate sensing by sampling from a database of previously sensed PDFs over objects and categories 
	def _getSensors(self,req):

		if not self.PDFsCreated:
			# create database of PDF samples
			numSamples = 10
			self.database = PDF_library(self.actionNames, self.numCategories, self.numObjects, numSamples)
			self.PDFsCreated = True

		#print req.actionID
		PDF = self.database.samplePDF(req.catID, req.actionID.val)
		
		return PDF

if __name__ == "__main__":

	server = robotTestServer()
	server.run()
