#!/bin/bash
for a in `seq 100`
do
    echo "RUN $a"
    rosrun gazebo_tools gazebo_model -z 0.01 -p robot_description spawn robot_description  
    sleep 1
    rosservice call delete_model robot_description
    sleep 1
done  
