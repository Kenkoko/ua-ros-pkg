#!/usr/bin/env python
import roslib; roslib.load_manifest('ua_audio_infomax')
import rospy, sys

############################################################################
# ROS node for InfoMax reinforcement learning with PyBrain
############################################################################

__author__ = 'Daniel Ford, dford@email.arizona.edu'

from agents import InfoMaxAgent

if __name__ == '__main__':

	# max number of steps to run
	maxSteps = 10

	episodes = 500		# number of learning episodes to run

	# main learning code that instantiates an agent
	# and communicates with the robot controller node to learn in the environment
	try:

		# grab keyword to choose learning algorithm or running
		option = sys.argv
		if len(option) is not 2:
			print "\nUsage is experimentWrapper.py [option]\n"
			print "Options: PGPE, CMAES, run (with most recently learned weights)\n"
			exit() 

		# run network with most recently learned weights
		if option[1] == "run":

			agent = InfoMaxAgent(option)
			agent.run(maxSteps)

		# train network
		else:
			agent = InfoMaxAgent(option)
			agent.train(episodes, maxSteps)
			agent.run(maxSteps)

	except rospy.ROSInterruptException: pass
