#!/usr/bin/env python
# Author: Daniel Hewlett
# This file/package was created as a simple example for CS 665
# at the University of Arizona, but is also a fully functional
# Text-to-Speech service wrapper for eSpeak.

PKG = 'espeak'

# All ROS python nodes should have these lines
import roslib; roslib.load_manifest(PKG)
import rospy

# This loads the definition of the service (Say.srv)
from espeak.srv import *

# This is needed to execute commands in the shell
import subprocess

#
# See the python service tutorial to understand what this code is doing
# http://www.ros.org/wiki/ROS/Tutorials/WritingServiceClient(python)
# 

# You can call the service from the command line with something like:
# rosservice call /say "Hello World"
class Speaker():
    def __init__(self):
        rospy.init_node('speaker', anonymous=True)
        s = rospy.Service('say', Say, self.handle_say)

    def handle_say(self, req):
        # Same as typing this:  espeak -v english "whatever the text is"
        # into the terminal
        subprocess.call(["espeak", "-v", "english", req.sentence]) 
        return []

if __name__ == '__main__':
    try:
        speaker = Speaker()
        rospy.spin()
    except rospy.ROSInterruptException: pass

