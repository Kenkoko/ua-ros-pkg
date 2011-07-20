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

from dynamixel_msgs.msg import JointState
from std_srvs.srv import Empty
from std_msgs.msg import Float64

if __name__ == '__main__':
    attribute_names = ['left_pressure', 'right_pressure', 'total_pressure',
                       'left_position', 'right_position',
                       'left_load', 'right_load',
                       'gripper_opening', 'ir_distance',
                       'left_velocity', 'right_velocity',
                       'left_goal', 'right_goal',
                       'left_ground_dist', 'right_ground_dist',
                       'has_grip']
                       
    attribute_types = ['C#'] * len(attribute_names)
    attribute_types[-1] = 'cD#'
    
    data = [0.0] * len(attribute_names)
    data[-1] = False  # no grip by default
    save = False
    
    rospy.init_node('gripper_data_collection', anonymous=True)
    
    def  process_float64_msgs(msg, idx):
        data[idx] = msg.data
        
    def process_joint_state(msg, idxs):
        data[idxs[0]] = msg.current_pos
        data[idxs[1]] = msg.load
        data[idxs[2]] = msg.velocity
        data[idxs[3]] = msg.goal_pos
        
    rospy.Subscriber('/left_finger_pressure', Float64, process_float64_msgs, 0)
    rospy.Subscriber('/right_finger_pressure', Float64, process_float64_msgs, 1)
    rospy.Subscriber('/total_pressure', Float64, process_float64_msgs, 2)
    rospy.Subscriber('/gripper_opening', Float64, process_float64_msgs, 7)
    rospy.Subscriber('/gripper_distance_sensor', Float64, process_float64_msgs, 8)
    rospy.Subscriber('/left_finger_controller/state', JointState, process_joint_state, [3,5,9,11])
    rospy.Subscriber('/right_finger_controller/state', JointState, process_joint_state, [4,6,10,12])
    rospy.Subscriber('/left_finger_ground_distance', Float64, process_float64_msgs, 13)
    rospy.Subscriber('/right_finger_ground_distance', Float64, process_float64_msgs, 14)
    
    def process_trigger(req):
        data[-1] = not data[-1]
        rospy.loginfo('Gripper %s holding %s' % ('IS' if data[-1] else 'IS NOT', 'SOMETHING' if data[-1] else 'ANYTHING'))
        return []
        
    rospy.Service('/has_grip_trigger', Empty, process_trigger)
    
    def process_start_stop(req):
        global save
        save = not save
        rospy.loginfo('Gripper data collection is %s' % ('STARTED' if save else 'STOPPED'))
        return []
        
    rospy.Service('/trigger_gripper_data_collection', Empty, process_start_stop)
    
    stamp = rospy.Time.now().to_sec()
    f = open('/tmp/%d%s' % (stamp, '.txt'), 'w')
    tmp = ['%s%s' % (attr_type,attr_name) for (attr_type,attr_name) in zip(attribute_types,attribute_names)]
    header = '\t'.join(tmp)
    f.write(header)
    f.write('\n')
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if save:
            sample = '\t'.join(map(str, data))
            f.write(sample)
            f.write('\n')
            f.flush()
        rate.sleep()
        
    f.close()

