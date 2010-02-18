#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Arizona Robotics Research Group,
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

import roslib
roslib.load_manifest('saliency_track')

import rospy
from geometry_msgs.msg import Point
from wubble_controllers.msg import JointStateList
from wubble_controllers.msg import PanTilt

class TrackSaliencyPOI():
    def __init__(self):
        self.hfov = 49.2
        self.vfov = 36.0
        self.center = Point(320, 240, 0)
        self.deg_pan_ratio = hfov / 640.0      # how many degrees in a pixel in x plane
        self.deg_tilt_ratio = vfov / 480.0       # how many degrees in a pixel in y plane
        self.pan_deg_ratio = 640.0 / hfov  # how many pixels in a degree in x plane
        self.tilt_deg_ratio = 480.0 / vfov  # how many pixels in a degree in y plane
        
        rospy.init_node('saliency_track_poi', anonymous=False)
        self.saliency_poi_sub = rospy.Subscriber('saliency_poi', Point, self.do_track_poi)
        self.move_head_pub = rospy.Publisher('camera_pan_tilt_controller/pan_tilt', PanTilt)
        self.joint_state_sub = rospy.Subscriber('camera_pan_tilt_controller/state', JointStateList, self.process_current_state)
        
        self.current_pan_angle = 0
        self.current_tilt_angle = 0
        self.current_pan_moving = False
        self.current_tilt_moving = False

    def process_current_state(self, state_list):
        pan_state = state_list.motor_states[0]
        tilt_state = state_list.motor_states[1]
        self.current_pan_angle = pan_state.angle
        self.current_tilt_angle = tilt_state.angle
        self.current_pan_moving = pan_state.moving
        self.current_tilt_moving = tilt_state.moving

    def do_track_poi(self, point):
        x = point.x
        y = point.y
        x_dist = x - self.center.x
        y_dist = -y + self.center.y
        x_dist_ang = x_dist * self.deg_pan_ratio
        y_dist_ang = y_dist * self.deg_tilt_ratio

        if not self.current_pan_moving and not self.current_tilt_moving:
#            if x_dist > 150 or y_dist > 70:
            self.move_head_pub.publish(self.current_pan_angle + x_dist_ang, self.current_tilt_angle + y_dist_ang)
        
if __name__ == '__main__':
    try:
        track = TrackSaliencyPOI()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    
