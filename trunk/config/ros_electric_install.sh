#!/bin/bash
echo "Installing ros-electric for Ubuntu 11.04"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu natty main" > /etc/apt/sources.list.d/ros-latest.list'

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install build-essential python-yaml cmake subversion git-core mercurial python-setuptools ros-electric-*
sudo easy_install -U rosinstall

echo "source /opt/ros/electric/setup.bash" >> ~/.bashrc
. ~/.bashrc

source /opt/ros/electric/setup.bash

mkdir ~/ros_workspace

echo "export ROS_PACKAGE_PATH=~/ros_workspace:$ROS_PACKAGE_PATH" >> ~/.bashrc
echo "export ROS_WORKSPACE=~/ros_workspace" >> ~/.bashrc
. ~/.bashrc

echo $ROS_PACKAGE_PATH

