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
roslib.load_manifest('ax12')

import rospy
from ax12.msg import Move

class Validator:
    
    def __init__(self):
        self.__publisher = rospy.Publisher('robot/ax12/commands', Move)
        rospy.Subscriber('robot/ax12/moves', Move, self.receivedMove)

    def receivedMove(self, data):
        """
        Do any validation of the Move object, then place the move on the
        "commands" topic.
        """
        # validate based on the min/max angles on the param server
        min = rospy.get_param('robot/ax12/motor/%d/minAngle' %data.id, 0)
        max = rospy.get_param('robot/ax12/motor/%d/maxAngle' %data.id, 0)
        
        if data.position > max:
            data.position = max
        elif data.position < min:
            data.position = min
        
        self.__publisher.publish(data)

if __name__ == '__main__':
    rospy.init_node('movevalidator', anonymous=True)
    Validator()
    print 'Validator started.\nUse ctrl + c to stop...'
    rospy.spin()