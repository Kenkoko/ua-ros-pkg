# Introduction #

This page tells you how to set up your account with a functional ROS installation and UA packages. These will only work on the class machines located in GS 930.

**NB: The machines in CS930 that have ROS installed all have names starting with "har". To check if a machine has ROS installed, run:**
```
ls -al /opt/ros/installed 
```

**NOTE:** On Monday, Jan 25th, the machines in CS 930 we upgraded to ROS 1.0. If you set up your account prior to that date, you will need to follow the un-installation instructions below, and then re-do the installation.

# Un-Installation #

  1. Execute the following commands to delete ros-related files:
```
rm -rf ~/ros
rm install_ros_user
rm rosconfig
rm .ros.bashrc
```
  1. You will need to edit your .bashrc file, and remove (what should be) the last two lines. You can do this with:
```
gedit ~/.bashrc
```
  1. The lines you are looking for are:
```
source ~/.ros.bashrc
export ROBOT=sim
```
  1. Make sure to close any open terminals so the old ros-related environment variables will be removed (they can interfere with the new installation).

# Installation #

Easy!
  1. Navigate to your home directory
```
cd ~
```
  1. Download the install script
```
wget http://ua-ros-pkg.googlecode.com/svn/trunk/config/install_ros_user
```
  1. Make it executable
```
chmod +x install_ros_user
```
  1. Run it!
```
./install_ros_user
```
  1. Close your terminal and re-open it, or else run:
```
source .bashrc
```
This is needed to load environment variables before you run any further ros commands.

# Testing #

Here's a simple demo to run to test that you've got everything working. Try running:
```
roslaunch wubble_mapping simulation.launch
```

This should launch the gazebo simulator, complete with the simulated Wubble robot in a small room, as well as the rviz visualization tool.

Also, try:
```
roslaunch pr2_gazebo pr2_empty_world.launch
```
This will launch a simulated PR2 (made by Willow Garage), a sophisticated humanoid robot. You can control the robot with the keyboard by launching (in another terminal):
```
roslaunch pr2_teleop teleop_keyboard.launch
```

If all of this worked, you're ready to get started with ROS! Take a look at the [ROS Quick Start Guide](http://www.ros.org/wiki/ROS/StartGuide) if you can't wait to explore ROS further.

If something didn't work, please send email to Daniel Hewlett.

# Updating #

The install script has checked out a copy of the ua-ros-pkg Google Code repository into your ~/ros folder. As this code is under active development, you may want to pull down the latest updates from the repository from time to time. Google Code simply hosts an SVN (Subversion) repository, which is simple to update:

  1. Navigate to the repository folder
```
cd ~/ros/ua-ros-pkg
```
  1. Run the update command
```
svn up
```

You won't be able to commit to the repository yet. When we start building code in team projects, we will provide a way to contribute to the repository.

# Building #

Take a quick look at [this ROS tutorial](http://www.ros.org/wiki/ROS/Tutorials/NavigatingTheFilesystem) to get an overview of packages and how the ROS file system is arranged. There are now two types of ROS packages on the machines you are using:
  1. Shared packages, which are part of the ROS distribution and come from Willow Garage. These have been previously installed (and compiled) in /opt/ros/installed, which means you can't change them.
  1. User packages, which live in the ros directory inside your home folder (specifically, ~/ros/ua-ros-pkg).

All packages are already built by the install script you executed above. After you make changes, you can rebuild the wubble stack using:
```
rosmake wubble
```