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

# Author: Daniel Hewlett

from wubble_mdp.msg import *
from math import pi
import numpy

angle_list = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
canonical_angles = {'E': 0, 'NE': (pi/4), 'N': (pi/2), 'NW': (3*pi/4), 
                    'W': pi, 'SW': -(3*pi/4), 'S': -(pi/2), 'SE': -(pi/4)} 
step = 0.5

def lookup(k, h):
    if h.has_key(k):
        return h[k]
    else:
        return None

def make_obj_state(name, state_tup, last_state_tup, class_map):
    state = MDPObjectState()
    state.mdp_class = class_map[name]
    state.name = name
    state.x = "%.1f" % state_tup[0]
    state.y = "%.1f" % state_tup[1]
    state.orientation = state_tup[2]
    if last_state_tup:
        state.last_x = "%.1f" % last_state_tup[0]
        state.last_y = "%.1f" % last_state_tup[1]
        state.last_orientation = last_state_tup[2]
    else:
        state.last_x = str(None)
        state.last_y = str(None)
        state.last_orientation = str(None)
    return state

def mdp_state_from_msg(state):
    mdp_state = {}
    last_mdp_state = {}
    
    for obj_state in state.object_states:
        mdp_state[obj_state.name] = (float(obj_state.x), float(obj_state.y), 
                                     obj_state.orientation)
        if obj_state.last_x:
            last_mdp_state[obj_state.name] = (float(obj_state.last_x), 
                                              float(obj_state.last_y), 
                                              obj_state.last_orientation)
    return [mdp_state, last_mdp_state]

## CLASSES ## 

def infer_class_from_name(name):
    if name.startswith('robot'):
        return 'robot'
    elif name.startswith('goal'):
        return 'location'
    elif name.startswith('object'):
        return 'object'

def get_class(state):
    if state.mdp_class == state.ROBOT_CLASS:
        return 'robot'
    elif state.mdp_class == state.OBJECT_CLASS:
        return 'object'
    elif state.mdp_class == state.LOCATION_CLASS:
        return 'location'
    
def make_class_map(mdp_state_msg):
    class_map = {}
    for obj_state in mdp_state_msg.object_states:
        class_map[obj_state.name] = get_class(obj_state)
    return class_map

def get_robot_name(class_map):
    for name, mdp_class in class_map.items():
        if mdp_class == 'robot':
            return name
    return None

## UTILS ##

def round_to_delta(n, delta=0.5):
    rem = numpy.mod(n, delta)
    flo = n - rem
    if rem > (delta/2):
        return numpy.round(flo + delta, 1)
    else:
        return numpy.round(flo, 1)

def convert_yaw(yaw):
    """Takes a yaw and converts to a N/S/E/W orientation"""
    if yaw >= 0:
        if yaw < (pi/8):
            return 'E'
        elif yaw < (3*pi/8):
            return 'NE'
        elif yaw < (5*pi/8):
            return 'N'
        elif yaw < (7*pi/8):
            return 'NW'          
        else:
            return 'W'
    else:
        if yaw > -(pi/8):
            return 'E'
        elif yaw > -(3*pi/8):
            return 'SE'
        elif yaw > -(5*pi/8):
            return 'S'
        elif yaw > -(7*pi/8):
            return 'SW'          
        else:
            return 'W'


