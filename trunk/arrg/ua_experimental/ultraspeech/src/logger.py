#!/usr/bin/env python

import os, sys

import roslib
roslib.load_manifest('ultraspeech')
import rospy

from ultraspeech.msg import Control, CurrentStim

class Logger:
    def __init__(self):
        self.logfile = None
        rospy.init_node("logger", anonymous=True)
        rospy.Subscriber("control", Control, self.control_cb)
        rospy.spin()
        
    def control_cb(self, data):
        timestamp = str(data.header.stamp.secs) + '.' + str(data.header.stamp.nsecs)
        runstatus = data.run
        directory = data.directory
        if self.logfile == None:
            logfilename = directory + '/logfile.txt'
            self.logfile = open(logfilename, 'w')
        self.logfile.write('control\ttime:%s\trun:%d\tdirectory:%s\n' %(timestamp, runstatus, directory))    
        
        
        
if __name__ == "__main__":
    Logger()
