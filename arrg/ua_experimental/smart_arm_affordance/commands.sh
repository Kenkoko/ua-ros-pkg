#!/bin/bash

commands_start() {
	echo -n ""
}

commands_reset_arm () {
	rostopic pub -1 finger_left_controller/command std_msgs/Float64 -- 1.0 > sometext.txt.nope
	rostopic pub -1 finger_right_controller/command std_msgs/Float64 -- 1.0 > sometext.txt.nope
	rostopic pub -1 /shoulder_tilt_controller/command std_msgs/Float64 -- 2.35 > sometext.txt.nope
    rostopic pub -1 /elbow_tilt_controller/command std_msgs/Float64 -- 2.0944 > sometext.txt.nope
	echo "reset complete"
}

commands_done () {
	rm -f sometext.txt.nope
	exit $1
}



if [ "$1" == "reset" ]; then
	commands_reset_arm
fi

commands_done 0
