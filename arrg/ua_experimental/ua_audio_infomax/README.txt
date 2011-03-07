******* notes on ua_audio_infomax code
******* March 6, 2011
******* Daniel Ford

******* contents of src directory

** experimentWrapper.py
test wrapper that will run PyBrain code in ROS
change number of steps, episodes, learning algorithm here
takes "PGPE" "CMAES" or "run" as a cmd line arg
running learning with "PGPE" or "CMAES" will create a trained network in InfoMaxNet.pkl
"run" will exercise the agent using the last trained network 

** agent.py - instantiates the PyBrain agent
** tasks.py - defines the PyBrain task
** environment.py - defines the PyBrain environment w/ ROS srvs for communication with robot node
uses InfoMax.srv definition in srv directory

** robotTestServer.py
creates a simulated robot node that communicates with the PyBrain environment thru ROS srvs
uses InfoMax.srv definition in srv directory

** PDF.py - classes used in robotTestServer to create simulated sensor results 

******** running the code
1. rosmake ua_audio_infomax to generate srv code 
2. launch roscore
3. launch simulated robot node: rosrun ua_audio_infomax robotTestServer.py
4. launch PyBrain node w/ PGPE: rosrun ua_audio_infomax experimentWrapper.py PGPE 
