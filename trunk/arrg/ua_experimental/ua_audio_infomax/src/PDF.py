from random import *
from scipy import *
from numpy import *

class PDF_library():
			
	def __init__(self, actionNames, numCategories, numObjects, numSamples):

		self.numObjects = numObjects
		self.numCategories = numCategories
		self.numSamples = numSamples	
		self.numActions = len(actionNames)
		self.PDF_database = []

		# create PDF database for all categories and actions
		self._createPDFs()
	
	# create numSamples PDFs per category and possible action
	def _createPDFs(self):

		for action in range(self.numActions):
			for cat in range(self.numCategories):
				for sample in range(self.numSamples):
					self.PDF_database.append(PDF(self.numCategories, cat, action))

	# sample randomly from PDFs for given object and action
	def samplePDF(self, catID, actionType):

		PDF = self.PDF_database[actionType*self.numCategories*self.numSamples + catID*self.numSamples + randint(0,self.numSamples-1)].PDF

		return PDF

class PDF():

	def __init__(self, numCategories, catID, actionType):

		# set probabilities of sensor returning the correct value
		graspProb = 52; liftProb = 58; dropProb = 70
		shakeProb = 73; placeProb = 67 
		probs = [graspProb, liftProb, dropProb, shakeProb, placeProb]

		# create blank PDF
		self.PDF = zeros(numCategories)

		# populate PDF according to object ID and action type
		actProb = (probs[actionType]/100.) - 0.1*random.random()

		# initialize PDF with small value to avoid zeros
		for i in range(len(self.PDF)):
			self.PDF[i] = 0.05 / len(self.PDF)	

		# create a crudely peaked PDF around the known category
		if (catID+1) > (numCategories-1):
			self.PDF[catID-1] += (1.-actProb-0.05)/2
			self.PDF[catID] += actProb
			self.PDF[0] += (1.-actProb-0.05)/2 			
		
		elif (catID-1) < 0:
			self.PDF[catID+1] += (1.-actProb-0.05)/2
			self.PDF[catID] += actProb
			self.PDF[numCategories-1] += (1.-actProb-0.05)/2 

		else:
			self.PDF[catID-1] += (1.-actProb-0.05)/2
			self.PDF[catID] += actProb
			self.PDF[catID+1] += (1.-actProb-0.05)/2

		# add some randomness and renormalize PDF
		for prob in range(len(self.PDF)):
			self.PDF[prob] += 0.08*random.random()
		sumtot = sum(self.PDF)
		for prob in range(len(self.PDF)):
			self.PDF[prob] /= sumtot 
