#!/usr/bin/env python
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


"""
test node that responds to service requests from the InfoMax agent
intended to mimic robot controller node
"""


import roslib; roslib.load_manifest('ua_audio_infomax')
import rospy

from std_srvs.srv import Empty

from ua_audio_infomax.PDF import PDF_library
from ua_audio_infomax.srv import InfoMax
from ua_audio_infomax.msg import Action as InfomaxAction


__author__ = 'Daniel Ford, Antons Rebguns'
__copyright__ = 'Copyright (c) 2011 Daniel Ford, Antons Rebguns'
__credits__ = 'Ian Fasel'

__license__ = 'BSD'
__maintainer__ = 'Daniel Ford'
__email__ = 'dford@email.arizona.edu'


class robotTestServer():
    def __init__(self):
        self.num_categories = 0
        self.object_names = []
        self.action_names = []
        self.num_objects = []
        self.current_location = 0
        self.pdf_database_initialized = False
        
        rospy.init_node('robotTestServer')
        rospy.Service('InfoMax', InfoMax, self.handle_infomax_request)
        rospy.Service('reset_current_location', Empty, self.reset_current_location)
        rospy.loginfo('Ready to run the robot')


    def initialize_pdf_library(self):
        rospy.loginfo('creating pdf library...')
        num_samples = 10
        self.database = PDF_library(self.action_names, self.num_categories, self.num_objects, num_samples)
        self.pdf_database_initialized = True
        rospy.loginfo('done')


    def reset_current_location(self, req):
        self.current_location = 0
        #rospy.loginfo('Reset requested, at location %d' % self.current_location)
        return []


    def _reset(self):
        self.reset_current_location(None)
        self.pdf_database_initialized = False


    def handle_infomax_request(self,req):
        """
        instruct the robot to move to or sense the selected object
        """
        self.num_categories = req.numCats
        self.object_names = req.objectNames
        self.num_objects = req.num_objects
        self.action_names = req.actionNames
        
        if not self.pdf_database_initialized: self.initialize_pdf_library()
        
        beliefs = None
        
        # perform action and get PDF (or None if we moved or reset the robot)
        if req.actionID.val == InfomaxAction.MOVE_LEFT:
            self.current_location = (self.current_location + 1) % self.num_objects
        elif req.actionID.val == InfomaxAction.MOVE_RIGHT:
            self.current_location = (self.current_location - 1) % self.num_objects
        else:
            beliefs = self.database.samplePDF(req.catID, req.actionID.val)
            
        #rospy.loginfo('At location %d', self.current_location)
        #rospy.loginfo('Performed %s', self.action_names[req.actionID.val])
        #rospy.loginfo('Sensed %s\n', str(beliefs))
        
        return beliefs, self.current_location


if __name__ == "__main__":
    server = robotTestServer()
    rospy.spin()

