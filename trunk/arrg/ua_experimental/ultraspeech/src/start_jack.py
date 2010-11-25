#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess, time, os

import roslib
roslib.load_manifest('ultraspeech')
import rospy

#from ultraspeech.msg import Control, CurrentStim
from std_msgs.msg import String



class Slave:
    def __init__(self):
	rospy.init_node('slave', anonymous=True)
	rospy.Subscriber("startup", String, self.startup_cb) 

	self.startJack()
	
	rospy.spin()
	
    def startup_cb(self, data):
	directory = data.data
	if (directory == 'done'):
	    self.ffado_pid.terminate()
	    self.jack_pid.terminate()
	    self.ffado_mixer_pid.terminate()

	else:
	    mic1dir = directory + '/mic1'
	    os.makedirs(mic1dir)
	    
	    mic2dir = directory + '/mic2'
	    os.makedirs(mic2dir)
	    
	    eggdir = directory + '/egg'
	    os.makedirs(eggdir)
	
    def startJack(self):
	jackstart = ['/usr/bin/jackd', '-r', '-p128', '-dfirewire', '-dhw:0', '-r96000', '-p1024', '-n3']
	self.jack_pid = subprocess.Popen(jackstart)
	time.sleep(3) # wait for jack to find the Saffire and turn it on

	ffadodbus = ['ffado-dbus-server']
	self.ffado_pid = subprocess.Popen(ffadodbus)
	time.sleep(1) # wait for ffado to start

	ffadomixer = ['ffado-mixer-qt3']
	self.ffado_mixer_pid = subprocess.Popen(ffadomixer)
	print "turn on phantom power\n"

if __name__ == "__main__":
    Slave()