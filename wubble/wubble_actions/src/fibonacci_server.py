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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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

PKG = 'learning_actionlib'
NAME = 'fibonacci_server'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionServer
from learning_actionlib.msg import *


class FibonacciServer():
    def __init__(self):
        self.result = FibonacciResult()
        self.feedback = FibonacciFeedback()
        self.feedback.sequence = list()
        self.feedback.sequence.append(0)
        self.feedback.sequence.append(1)
        self.server = SimpleActionServer("fibonacci", FibonacciAction, self.execute_callback)   


    def execute_callback(self, goal):
        r = rospy.Rate(1)
        success = True
        rospy.loginfo("%s: Executing, creating fibonacci sequence of order %s with seeds %s, %s", \
                        NAME, goal.order, self.feedback.sequence[0], self.feedback.sequence[1]);

        for i in range(1, goal.order + 1):
            if self.server.is_preempt_requested():
                rospy.loginfo("%s: Preempted", NAME)
                self.server.set_preempted()
                success = False
                break
            self.feedback.sequence.append(self.feedback.sequence[i] + self.feedback.sequence[i - 1])
            self.server.publish_feedback(self.feedback)
            r.sleep()

        if (success):
            self.result.sequence = self.feedback.sequence
            rospy.loginfo("%s: Succeeded", NAME)
            self.server.set_succeeded(self.result)


if __name__ == '__main__':
    try:
        rospy.init_node(NAME)
        s = FibonacciServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

