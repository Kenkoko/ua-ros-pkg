#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Arizona Robotics Research Group,
# University of Arizona. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Antons Rebguns

import roslib
roslib.load_manifest('saliency_tracking')

import rospy
import math
from geometry_msgs.msg import Point
from pr2_controllers_msgs.msg import JointControllerState
from std_msgs.msg import Float64

class TrackSaliencyPOI():
    def __init__(self):
        self.hfov = 49.2
        self.vfov = 36.0
        self.center = Point(320, 240, 0)
        self.deg_pan_ratio = self.hfov / 640.0       # how many degrees in a pixel in x plane
        self.deg_tilt_ratio = self.vfov / 480.0      # how many degrees in a pixel in y plane
        self.pan_deg_ratio = 640.0 / self.hfov       # how many pixels in a degree in x plane
        self.tilt_deg_ratio = 480.0 / self.vfov      # how many pixels in a degree in y plane
        
        rospy.init_node('saliency_track_poi', anonymous=False)
        self.saliency_poi_sub = rospy.Subscriber('saliency_poi', Point, self.do_track_poi)
        self.move_head_pan_pub = rospy.Publisher('head_pan_controller/command', Float64)
        self.move_head_tilt_pub = rospy.Publisher('head_tilt_controller/command', Float64)
        self.joint_state_sub = rospy.Subscriber('head_pan_controller/state', JointControllerState, self.process_current_pan_state)
        self.joint_state_sub = rospy.Subscriber('head_tilt_controller/state', JointControllerState, self.process_current_tilt_state)
        
        self.current_pan_angle = 0
        self.current_tilt_angle = 0
        self.current_pan_moving = False
        self.current_tilt_moving = False

    def process_current_pan_state(self, new_state):
        self.current_pan_angle = math.degrees(new_state.process_value)

    def process_current_tilt_state(self, new_state):
        self.current_tilt_angle = math.degrees(new_state.process_value)

    def do_track_poi(self, point):
        x = point.x
        y = point.y
        x_dist = -x + self.center.x
        y_dist = -y + self.center.y
        x_dist_ang = x_dist * self.deg_pan_ratio
        y_dist_ang = y_dist * self.deg_tilt_ratio

        self.move_head_pan_pub.publish(math.radians(self.current_pan_angle + x_dist_ang))
        self.move_head_tilt_pub.publish(math.radians(self.current_tilt_angle + y_dist_ang))
        
if __name__ == '__main__':
    try:
        track = TrackSaliencyPOI()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    
