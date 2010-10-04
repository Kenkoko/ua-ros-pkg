#! /usr/bin/env python

PKG = 'wubble_mdp'
NAME = 'test_simulation'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionClient

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import *
from simulator_state.msg import *
from wubble_mdp.srv import *
from time_series.srv import *

from math import pi
import numpy
from tf.transformations import quaternion_about_axis, euler_from_quaternion

from wubble_mdp.state import *
from wubble_mdp.relations import *

rospy.init_node(NAME)

constrain = rospy.ServiceProxy('time_series/constrain_verb', ConstrainVerb)
#constrain("go", ["Contact(thing,obstacle)"])

simulate = rospy.ServiceProxy('environment/simulate_action', SimulateAction)
update = rospy.ServiceProxy('time_series/fsm_update', FSMUpdate)

state = MDPState()
robot_state = MDPObjectState(mdp_class='robot', name='robot_description', x='0.0', y='0.0', orientation='E')
goal_state = MDPObjectState(mdp_class='location', name='goal_test', x='4.0', y='0.0', orientation='E')
state.object_states = [robot_state, goal_state]

for i in range(8):
    res = simulate(state, 'forward')
    state = res.state
   
    #print state
 
    prop_list = []
    for rel in state.relations:
        relation = parse_rel(rel)
        if relation[1]:
            prop_list.append(relation[0].replace('robot_description', 'thing').replace('goal_test', 'place'))
    print prop_list
    print update('go', prop_list)
    
   
    