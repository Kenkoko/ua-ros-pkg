from random import *

class PDF_library():
			
	def __init__(self, actionNames, numObjects, numSamples):

		self.numObjects = numObjects
		self.numSamples = numSamples	
		self.numActions = len(actionNames)-3	# subtract move and reset actions
		self.PDF_database = []

		# create PDF database for all objects and actions
		for action in range(self.numActions):
			self._createPDFs(action)
	
	# create numSamples PDFs per object and possible
	def _createPDFs(self, actionType):

		for obj in range(self.numObjects):
			for sample in range(self.numSamples):
				self.PDF_database.append(PDF(self.numObjects, obj, actionType))

	# sample randomly from PDFs for given object and action
	def samplePDF(self, objectID, actionType):

		PDF = self.PDF_database[actionType*self.numObjects*self.numSamples+objectID*self.numSamples+randint(0,self.numSamples-1)].PDF

		return PDF

class PDF():

	def __init__(self, numObjects, objectID, actionType):

		# set probabilities of sensor returning the correct value
		pickUpProb = 85; dropProb = 80
		pushProb = 90; squeezeProb = 73 
		probs = [pickUpProb, dropProb, pushProb, squeezeProb]

		# create blank PDF
		self.PDF = []
		for i in range(numObjects):
			self.PDF.append(0)

		# populate PDF according to object ID and action type
		actProb = (probs[actionType]/100.) - 0.1*random()

		if (objectID+1) > (numObjects-1):
			self.PDF[objectID-1] = (1.-actProb)/2
			self.PDF[objectID] = actProb
			self.PDF[0] = (1.-actProb)/2 			
		
		elif (objectID-1) < 0:
			self.PDF[objectID+1] = (1.-actProb)/2
			self.PDF[objectID] = actProb
			self.PDF[numObjects-1] = (1.-actProb)/2 

		else:
			self.PDF[objectID-1] = (1.-actProb)/2
			self.PDF[objectID] = actProb
			self.PDF[objectID+1] = (1.-actProb)/2 
