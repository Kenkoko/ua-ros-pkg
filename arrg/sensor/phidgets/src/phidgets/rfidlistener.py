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
roslib.load_manifest('phidgets')

import rospy
from phidgets.msg import RFIDEvent

# 15007ED062 - 1
# 15007EC394 - 2
# 15007EC398 - 1
# 15007EB2F7 - 2
# 15007EC21A - 1

block_id = {'15007ED062' : 1,
            '15007EC394' : 2,
            '15007EC398' : 1,
            '15007EB2F7' : 2,
            '15007EC21A' : 1
            }

def printBig1():
    print """
           1
          11
         111
        1 11
          11
          11
          11
          11
          11
       1111111
"""

def printBig2():
    print """
        222222
      22      22
             22
            22
           22
          22
         22
        22
       22
      222222222
"""

def callback(data):
    if data.gained == 1:
        try:
            val = block_id[data.tag]
            if val == 1:
                printBig1()
            else:
                printBig2()
        except KeyError, e:
            print 'Unidentified Object'
    else:
        print "\n"*100
 
def listener():
    rospy.init_node('rfid_listener', anonymous=True)
    rospy.Subscriber("rfid", RFIDEvent, callback)
    print 'Ready for RFID scanning.'
    rospy.spin()

if __name__ == '__main__':
    listener()
