# -*- coding: utf-8 -*-
#
# Copyright (c) 2011, Daniel Ford, Antons Rebguns
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# Neither the name of the <ORGANIZATION> nor the names of its contributors may
# be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import pickle
from numpy.random import randint


__author__ = 'Daniel Ford, Antons Rebguns'
__copyright__ = 'Copyright (c) 2011 Daniel Ford, Antons Rebguns'
__credits__ = 'Ian Fasel'

__license__ = 'BSD'
__maintainer__ = 'Daniel Ford'
__email__ = 'dford@email.arizona.edu'


class PDF_library():
    def __init__(self, actionNames, numCategories, numObjects, numSamples):
        self.numObjects = numObjects
        self.numCategories = numCategories
        self.numSamples = numSamples
        self.numActions = len(actionNames)
        self.PDF_database = []
        
        # create PDF database for all categories and actions
        self._createPDFs()


    def _createPDFs(self):
        """
        create numSamples PDFs per category and possible action
        """
        for action in range(self.numActions):
            if action in [7,8]: continue
            for cat in range(self.numCategories):
                for sample in range(self.numSamples):
                    self.PDF_database.append(PDF(self.numCategories, cat, action, sample))


    def samplePDF(self, catID, actionType):
        """
        sample randomly from PDFs for given object and action
        """
        # select a PDF at random from the correct category
        PDF = self.PDF_database[actionType*self.numCategories*self.numSamples + catID*self.numSamples + randint(self.numSamples)].PDF
        return PDF


class PDF():
    def __init__(self, numCategories, catID, actionType, sample):
        self.action_names = ['grasp',        # 0
                             'lift',         # 1
                             'drop',         # 2
                             'shake_roll',   # 3
                             'place',        # 4
                             'push',         # 5
                             'shake_pitch',  # 6
                             'move_left',    # 7
                             'move_right',   # 8
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
        
        self.PDF = [object_pdfs[sample][obj] for obj in self.object_names]

