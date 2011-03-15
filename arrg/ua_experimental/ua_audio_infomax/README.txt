############## notes on ua_audio_infomax code
############## March 14, 2011
############## Daniel Ford

############## contents of src directory

-- tasks.py
defines the PyBrain task
set number of time-horizon RBF kernels here

-- environment.py
defines the PyBrain environment w/ ROS srvs for communication with robot node

-- robotTestServer.py
creates a simulated robot node
accepts action (sensing) request, returns PDF over categories given that sensing action

-- PDF.py
classes used in robotTestServer to create simulated sensor results 

############## previous and deprecated source

-- experimentWrapper.py -- *just utility code as of now*
test wrapper that will run PyBrain code in ROS
set number of steps per episodes, number of episodes, and learning algorithm here

takes "PGPE" "CMAES" or "run" as a cmd line argument
"PGPE" or "CMAES" will learn using the selected algo, save the trained network as infomaxNet.pkl, then run using the trained network
"run" will exercise the agent using the trained network in infomaxNet.pkl 

"State pre" and "State post" list conditional PDFs per action over the categories
appended at the end are RBF kernels based on the number of steps (actions)

-- agent.py -- *only used with experimentWrapper.py*
instantiates the PyBrain agent
set number of categories, object names and categories, action names here

-- agents.py -- *deprecated*
additional InfoMax agents, not working

############## running the code
1. rosmake ua_audio_infomax, launch roscore
2. rosrun ua_audio_infomax robotTestServer.py (starts simulated robot)
3. rosrun ua_audio_infomax experimentGraphingWrapper.py (starts learning node, saves and plots data when done) 
