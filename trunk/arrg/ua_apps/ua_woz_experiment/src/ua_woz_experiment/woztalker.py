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
roslib.load_manifest('ua_woz_experiment')

import sys
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    
    try:
        chatter = rospy.Publisher('woz/chatter', String)
        rospy.init_node('woztalker', anonymous=True)

        commands = {"0":"Hello!",
                    "1":"One",
                    "2":"Two",
                    "3":"Many",
                    "4":"OK",
                    "5":"Yes",
                    "6":"No",
                    "7":"I didn't understand",
                    "8":"Please wait, I am crunching numbers.",
                    "9":"I am done",
                    "p":"Purple",
                    "b":"Blue/Cyan",
                    "t":"Triangle",
                    "s":"Square",
                    "y":"Thank you"}

        print 'To respond, please choose one of the following: '
        for c in sorted(commands):
            print c + " -- " + commands[c]
        print 'At the end of each phrase press return.'

        while not rospy.is_shutdown():
            try:
                cmd = sys.stdin.readline().strip().lower()
                if cmd in commands:
                  chatter.publish(String(commands[cmd]))
                  print commands[cmd]
                else :
                   print 'Invalid phrase.'
            except:
                continue
    except rospy.ROSInterruptException: pass
