#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Antons Rebguns
#

import roslib; roslib.load_manifest('wubble_blocks')
import rospy

from wubble_blocks.srv import ClassifyObject
from object_tracking.srv import GetTracksInterval

class ObjectClassifier:
    def __init__(self):
        self.classify_obj_server = rospy.Service('classify_object', ClassifyObject, self.classify)
        self.get_tracks_srv = rospy.ServiceProxy('get_tracks_interval', GetTracksInterval)
        rospy.wait_for_service('get_tracks_interval')
    
    def classify(self, req):
        resp = self.get_tracks_srv(req.id, req.swat_time, req.swat_time + rospy.Duration(5.0))
        return self.categorize_track(resp.tracks[0])
    
    def categorize_track(self, track):
        # TODO: put actual classification algorithm here
        return "Ball"

if __name__ == '__main__':
    try:
        rospy.init_node('object_classifier', anonymous=True)        
        swat = ObjectClassifier()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
