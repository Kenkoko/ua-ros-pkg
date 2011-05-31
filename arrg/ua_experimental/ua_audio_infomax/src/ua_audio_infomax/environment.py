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


import numpy as np

import roslib; roslib.load_manifest('ua_audio_infomax')
import rospy

from std_srvs.srv import Empty

from pybrain.utilities import Named
from pybrain.rl.environments.environment import Environment

from ua_audio_infomax.robotTestServer import robotTestServer
from ua_audio_infomax.msg import Action as InfomaxAction
from ua_audio_infomax.srv import InfoMax
from ua_audio_infomax.srv import InfoMaxRequest


__author__ = 'Daniel Ford, Antons Rebguns'
__copyright__ = 'Copyright (c) 2011 Daniel Ford, Antons Rebguns'
__credits__ = 'Ian Fasel'

__license__ = 'BSD'
__maintainer__ = 'Daniel Ford'
__email__ = 'dford@email.arizona.edu'


class InfoMaxEnv(Environment, Named):
    def __init__(self, category_names, action_names, num_objects, standalone_robot_server=True):
        self.category_names = category_names
        self.action_names = action_names
        
        self.num_categories = len(category_names)
        self.num_objects = num_objects
        
        if standalone_robot_server:
            self.robot_server = None
            rospy.loginfo('waiting for InfoMax service...')
            rospy.wait_for_service('InfoMax')
            rospy.loginfo('connected to InfoMax service')
            self.get_belief_distribution = rospy.ServiceProxy('InfoMax', InfoMax)
            
            rospy.loginfo('waiting for reset_current_location service...')
            rospy.wait_for_service('reset_current_location')
            rospy.loginfo('connected to reset_current_location service')
            self.reset_current_location = rospy.ServiceProxy('reset_current_location', Empty)
        else:
            self.robot_server = robotTestServer(standalone=False)
            
        self.objects = None
        self.reset()


    def reset(self, randomize=True):
        """
        Shuffles objects at all environment locations.
        """
        if randomize:
            self.objects = np.random.randint(0, self.num_categories, self.num_objects)
            
        if not self.robot_server: self.reset_current_location(Empty())
        else: self.robot_server.reset_current_location(Empty())
        
        self.current_location = 0
        self.current_state = 'init'


    def sense(self, action):
        """
        Perform an action at current location and get the belief distribution
        over all categories.
        """
        try:
            req = InfoMaxRequest()
            req.num_objects = self.num_objects
            req.objectNames = self.category_names
            req.actionNames = self.action_names
            req.numCats = self.num_categories
            req.catID = self.objects[self.current_location]
            req.actionID.val = action
            
            rospy.logdebug('calling sense service with action %d (%s) on object %d(%s)' %
                           (req.actionID.val, self.action_names[req.actionID.val], req.catID, self.category_names[req.catID]))
                           
            if not self.robot_server: res = self.get_belief_distribution(req)
            else: res = self.robot_server.handle_infomax_request(req)
            self.current_location = res.location
            self.current_state = res.state
            
            return res
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            return None

