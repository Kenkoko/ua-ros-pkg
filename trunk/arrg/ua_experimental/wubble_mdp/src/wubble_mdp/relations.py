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
from simulator_state.msg import *

from wubble_mdp.state import *

from math import pi, atan2, sin, cos
import numpy

use_fake_contact = False

def compute_initial_relations(mdp_state, class_map):
    """Each argument is a hash"""
    relations = []    

    for name, state in mdp_state.items():
        relations.append(SimpleRelation(rel_name="Forward", obj_names=[name], value=False))
        relations.append(SimpleRelation(rel_name="Turned", obj_names=[name], value=False))
        relations.append(SimpleRelation(rel_name="Moved", obj_names=[name], value=False))
        relations.append(SimpleRelation(rel_name="Left", obj_names=[name], value=False))
        relations.append(SimpleRelation(rel_name="Right", obj_names=[name], value=False))  

    if len(mdp_state.keys()) > 1:
        relations += compute_binary_relations(mdp_state, {}, class_map)

    return relations

def compute_relations(mdp_state, last_mdp_state, class_map, simulation=False):
    """Each argument is a hash"""
    relations = []    

    for name, state in mdp_state.items():
        last_state = lookup(name, last_mdp_state)

        print name, state, last_state

        if last_state:
            # Forward
            moved = (state[0] <> last_state[0] or state[1] <> last_state[1])
            relations.append(SimpleRelation(rel_name="Forward", obj_names=[name], value=moved))
            
            # Turned
            turned = (state[2] <> last_state[2])
            relations.append(SimpleRelation(rel_name="Turned", obj_names=[name], value=turned))

            # Moved
            relations.append(SimpleRelation(rel_name="Moved", obj_names=[name], value=(turned or moved)))

            if turned:
                diff = delta_angle(canonical_angles[state[2]], canonical_angles[last_state[2]])
                left = (diff > 0)
                relations.append(SimpleRelation(rel_name="Left", obj_names=[name], value=left))
                relations.append(SimpleRelation(rel_name="Right", obj_names=[name], value=(not left)))  
            else:  
                relations.append(SimpleRelation(rel_name="Left", obj_names=[name], value=False))
                relations.append(SimpleRelation(rel_name="Right", obj_names=[name], value=False))  
        else:
            relations.append(SimpleRelation(rel_name="Forward", obj_names=[name], value=False))
            relations.append(SimpleRelation(rel_name="Turned", obj_names=[name], value=False))
            relations.append(SimpleRelation(rel_name="Moved", obj_names=[name], value=False))
            relations.append(SimpleRelation(rel_name="Left", obj_names=[name], value=False))
            relations.append(SimpleRelation(rel_name="Right", obj_names=[name], value=False)) 

    if len(mdp_state.keys()) > 1:
        relations += compute_binary_relations(mdp_state, last_mdp_state, class_map, simulation)

    return relations

def compute_binary_relations(mdp_state, last_mdp_state, class_map, simulation=False):
    """Returns the list of binary relations"""
    relations = []
    
    seq = kv_sequence(mdp_state)
    for s in seq:
        s.append(lookup(s[0], last_mdp_state))

    # For symmetric relations, we only want to compute them once
    # If we need non-symmetric relations, need to do them in another loop
    for i in range(len(seq)):
        for j in range(i+1, len(seq)):
            first = seq[i]
            second = seq[j]

            c_dist = chess_distance(first[1], second[1])
            #if simulation:
            relations.append(SimpleRelation(rel_name="Contact",
                                            obj_names=[second[0], first[0]],
                                            value=(c_dist == 0)))

            if first[2] and second[2]: # i.e., both have a last state
                
                old_c_dist = chess_distance(first[2], second[2])                
                        
                relations.append(SimpleRelation(rel_name="DistanceDecreased", 
                                                obj_names=[first[0], second[0]],
                                                value=(c_dist < old_c_dist)))
                relations.append(SimpleRelation(rel_name="DistanceConstant", 
                                                obj_names=[first[0], second[0]],
                                                value=(c_dist == old_c_dist)))
                relations.append(SimpleRelation(rel_name="DistanceIncreased", 
                                                obj_names=[first[0], second[0]],
                                                value=(c_dist > old_c_dist)))
            else:
                print ">>>>>>>>> THERE IS NO LAST STATE THIS MUST BE INITIAL GET_STATE CALL"
                relations.append(SimpleRelation(rel_name="DistanceDecreased", 
                                                obj_names=[first[0], second[0]],
                                                value=False))
                relations.append(SimpleRelation(rel_name="DistanceConstant", 
                                                obj_names=[first[0], second[0]],
                                                value=False))
                relations.append(SimpleRelation(rel_name="DistanceIncreased", 
                                                obj_names=[first[0], second[0]],
                                                value=False))

    robot_index = find_robot_index(seq, class_map)
    if robot_index == -1:
        print "JJSJ:FLJSDLJFDJF"
    robot_state = seq[robot_index]
    for i in range(0,len(seq)): 
        if i <> robot_index:
            relations += make_angle_relations(robot_state[0], seq[i][0], 
                                              robot_state[1], seq[i][1])
            
    return relations

def find_robot_index(seq, class_map):
    for index, state in enumerate(seq):
        if class_map[state[0]] == 'robot':
            return index
    return -1

def compute_rel_angle(robot_state, object_state):
    r_yaw = canonical_angles[robot_state[2]]
    r_pos = robot_state[0:2]
    o_pos = object_state[0:2]
    
    diff_vec = numpy.subtract(o_pos, r_pos)

    direct_angle = atan2(diff_vec[1], diff_vec[0])

    return delta_angle(r_yaw, direct_angle)

def make_angle_relations(name1, name2, state1, state2):
    rels = ["RightOf", "LeftOf", "InFrontOf", "Behind"]
    true_rel = None

    if chess_distance(state1, state2) > 0: # TODO: Turn this back on for next real training example
        rel_angle = compute_rel_angle(state1, state2)

        index = 0
        new_angle = rel_angle
        if rel_angle < 0:
            index = 1
            new_angle = abs(new_angle)

        if new_angle < (pi/4):
            true_rel = rels[2]
        elif new_angle < (3*pi/4):
            true_rel = rels[index]
        else:
            true_rel = rels[3]

    relations = []    
    for rel in rels:
        relations.append(SimpleRelation(rel_name=rel, 
                                        obj_names=[name1, name2], 
                                        value=(rel == true_rel)))

    return relations

def shift_angle(theta):
    if theta < 0:
        return theta + (2*pi)    
    else:
        return theta % (2*pi)

def chess_distance(my_state, your_state):
    dist = max(abs(my_state[0] - your_state[0]), 
               abs(my_state[1] - your_state[1]))
    return int(dist / step)

def delta_angle(a, b):
    return atan2(sin(a-b), cos(a-b)) # Trig FTW

def kv_sequence(di):
    return [[k,di[k]] for k in sorted(di.keys())] # Actually, list comprehensions FTW

def parse_rel(rel):
    return [rel.rel_name + "(" + ",".join(rel.obj_names) + ")", rel.value]












