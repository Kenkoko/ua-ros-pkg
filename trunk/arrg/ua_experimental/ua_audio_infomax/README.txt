############## notes on ua_audio_infomax code
############## March 14, 2011
############## Daniel Ford

############## running the code

1. (first time only) rosmake ua_audio_infomax
2. launch roscore
3. rosrun ua_audio_infomax robotTestServer.py (starts simulated robot)
4. rosrun ua_audio_infomax experimentGraphingWrapper.py (starts learning node, saves and plots data when done)
5. (from /src/data) python ../graphExperiment.py [timestamp] (optional, to view plots for a given experiment) 

NOTE: pkl data files are currently created in the directory you're in when performing step 4 - can't sort out permissions issues that blocks folder creation

############## notes on current source 

## experimentGraphingWrapper.py
main script
runs and tests learning, saves experiment data, calls graphExperiment.py to plot results
each run creates a timestamped folder in src/data with pkl result files and experiment.desc, which saves the experiment and policy parameters 

## tasks.py
PyBrain task 
set number of time-horizon RBF kernels here

## environment.py
PyBrain environment w/ ROS srvs

## robotTestServer.py
creates a simulated robot node
accepts action (sensing) request, returns PDF over categories given that sensing action

## PDF.py
classes used in robotTestServer to create simulated sensor results 

## graphExperiment.py
graphing functions, called from experimentGraphingWrapper.py
can be called directly from /src with "python ../graphExperiment.py [timestamp]"

## PlotInfoMaxExample.py
helper functions for graphExperiment.py

############## old/deprecated

experimentWrapper.py
agent.py
agents.py
