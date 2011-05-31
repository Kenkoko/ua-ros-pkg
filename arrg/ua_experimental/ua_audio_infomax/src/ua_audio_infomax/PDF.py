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
    def __init__(self, action_names, object_names):
        self.action_names = action_names
        self.object_names = object_names
        
        # create PDF database for all categories and actions
        self.read_pdf_database()


    def read_pdf_database(self):
        pdfs_in = open('/tmp/obj_pdf.pkl', 'rb')
        self.pdf_database = pickle.load(pdfs_in)
        pdfs_in.close()


    def sample(self, action_id, category_id):
        """
        sample randomly from PDFs for given object and action
        """
        # select a PDF at random from the correct category
        action_name = self.action_names[action_id]
        category_name = self.object_names[category_id]
        
        pdf_samples = self.pdf_database[action_name][category_name]
        pdf = pdf_samples[randint(len(pdf_samples))]
        
        # translate from map to list representation
        return [pdf[obj] for obj in self.object_names]


    def read_pdf_database_new(self):
        pdfs_in = open('/tmp/proportions.pkl', 'rb')
        self.pdf_database, _, _ = pickle.load(pdfs_in)
        pdfs_in.close()


    def sample_new(self, action_id, category_id):
        """
        sample randomly from PDFs for given object and action
        """
        # select a PDF at random from the correct category
        action_name = self.action_names[action_id]
        category_name = self.object_names[category_id]
        
        pdf_samples = self.pdf_database[action_name][category_name]
        return pdf_samples[randint(pdf_samples.shape[0])].tolist()

