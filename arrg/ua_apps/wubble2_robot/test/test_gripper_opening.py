#! /usr/bin/env python

# Copyright (c) 2010, Antons Rebguns
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
# Author: Antons Rebguns
#

import roslib; roslib.load_manifest('wubble2_robot')
import rospy

import math
from tf import TransformListener

if __name__ == '__main__':
    rospy.init_node('gripper_opening', anonymous=True)
    listener = TransformListener()
    start = rospy.Time.now().to_sec()
    listener.waitForTransform('/L7_wrist_roll_link', '/left_fingertip_link', rospy.Time(0), rospy.Duration(0.1))
    middle = rospy.Time.now().to_sec()
    listener.waitForTransform('/L7_wrist_roll_link', '/right_fingertip_link', rospy.Time(0), rospy.Duration(0.1))
    end = rospy.Time.now().to_sec()
    print middle-start,end-start
    
    start = rospy.Time.now().to_sec()
    l_pos,l_ori = listener.lookupTransform('/L7_wrist_roll_link', '/left_fingertip_link', rospy.Time(0))
    middle = rospy.Time.now().to_sec()
    r_pos,r_ori = listener.lookupTransform('/L7_wrist_roll_link', '/right_fingertip_link', rospy.Time(0))
    end = rospy.Time.now().to_sec()
    print middle-start,end-start
    
    dx = l_pos[0] - r_pos[0]
    dy = l_pos[1] - r_pos[1]
    dz = l_pos[2] - r_pos[2]
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    #print l_pos, 'a', l_ori
    #print r_pos, 'a', r_ori
    #print dx, dy, dz
    print dist

