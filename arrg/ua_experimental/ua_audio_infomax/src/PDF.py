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
        self.randomCount = 0

        # create PDF database for all categories and actions
        self._createPDFs()
    
    # create numSamples PDFs per category and possible action
    def _createPDFs(self):

        for action in range(self.numActions):
            for cat in range(self.numCategories):
                for sample in range(self.numSamples):
                    self.PDF_database.append(PDF(self.numCategories, cat, action, sample))

    # sample randomly from PDFs for given object and action
    def samplePDF(self, catID, actionType):
        randomMeasure = 1

        # select a PDF at random from the correct category
#        if self.randomCount < randomMeasure: 
        PDF = self.PDF_database[actionType*self.numCategories*self.numSamples + catID*self.numSamples + randint(0,self.numSamples-1)].PDF
#            self.randomCount += 1
        # except every randomMeasure draws, select a PDF from a random category (could include the right category)
#        else:
#            PDF = self.PDF_database[actionType*self.numCategories*self.numSamples + randint(0,self.numCategories-1)*self.numSamples + randint(0,self.numSamples-1)].PDF
#            self.randomCount = 0
        return PDF


import pickle

class PDF():
    def __init__(self, numCategories, catID, actionType, sample):
        self.action_names = ['grasp',        # 0
                             'lift',         # 1
                             'drop',         # 2
                             'shake_roll',   # 3
                             'place',        # 4
                             'push',         # 5
                             'shake_pitch',  # 6
                            ]
                            
        self.object_names = ['pink_glass',           # 0
                             'german_ball',          # 1
                             'blue_cup',             # 2
                             'blue_spiky_ball',      # 3
                             'screw_box',            # 4
                             'wire_spool',           # 5
                             'sqeaky_ball',          # 6
                             'duck_tape_roll',       # 7
                             'ace_terminals',        # 8
                             'chalkboard_eraser',    # 9
                            ]
                            
        pdfs_in = open('/tmp/obj_pdf.pkl', 'rb')
        pdf_map = pickle.load(pdfs_in)
        pdfs_in.close()
        
        # get all object distributions for a given action
        action_pdfs = pdf_map[self.action_names[actionType]]
        object_pdfs = action_pdfs[self.object_names[catID]]
        
        self.PDF = []
        num_zeros = 0
        for obj in self.object_names:
            #print 'processing', object_pdfs[sample]
            try:
                self.PDF.append(object_pdfs[sample][obj])
            except KeyError as k:
                print obj, 'not found, replacing with 0'
                self.PDF.append(0)
                num_zeros += 1
                
        for i in range(len(self.PDF)):
            if self.PDF[i] == 0:
                self.PDF[i] = (1.0-sum(self.PDF))/num_zeros
            
#        print self.action_names[actionType], self.object_names[catID], self.PDF
        
#        # set probabilities of sensor returning the correct value
#        graspProb = 52; liftProb = 58; dropProb = 70
#        shakeProb = 73; placeProb = 67 
#        probs = [graspProb, liftProb, dropProb, shakeProb, placeProb]

#        # create blank PDF
#        self.PDF = zeros(numCategories)

#        # populate PDF according to object ID and action type
#        actProb = (probs[actionType]/100.) - 0.1*random.random()

#        # spread probabilities around
#        for i in range(len(self.PDF)):
#            self.PDF[i] = 1/len(self.PDF)

#        self.PDF[catID] += 0.2*random.random()

#        """
#        # initialize PDF with small value to avoid zeros
#        for i in range(len(self.PDF)):
#            self.PDF[i] = 0.05 / len(self.PDF)    

#        # create a crudely peaked PDF around the known category
#        if (catID+1) > (numCategories-1):
#            self.PDF[catID-1] += (1.-actProb-0.05)/2
#            self.PDF[catID] += actProb
#            self.PDF[0] += (1.-actProb-0.05)/2
#        
#        elif (catID-1) < 0:
#            self.PDF[catID+1] += (1.-actProb-0.05)/2
#            self.PDF[catID] += actProb
#            self.PDF[numCategories-1] += (1.-actProb-0.05)/2 

#        else:
#            self.PDF[catID-1] += (1.-actProb-0.05)/2
#            self.PDF[catID] += actProb
#            self.PDF[catID+1] += (1.-actProb-0.05)/2
#        """

#        # add some randomness and renormalize PDF
#        for prob in range(len(self.PDF)):
#            self.PDF[prob] += 0.08*random.random()
#        sumtot = sum(self.PDF)
#        for prob in range(len(self.PDF)):
#            self.PDF[prob] /= sumtot
