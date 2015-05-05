# ROS Instructions #

  1. Remove any and everything ROS related (~/ros, anything like ~/.ros, rosinstall, ROS-related lines from ~/.bashrc, all of it!). After you're done, close all open terminal windows.
  1. Open up this page: http://www.ros.org/wiki/ROS/Installation/Ubuntu and perform steps 1.1 and 1.2.
  1. Open up synaptic package manager with root permissions
```
sudo synaptic
```
  1. Hit the "Reload" button to load the ROS repository for the first time
  1. Hit the "Origin" button at the bottom left to show a list of repositories.
  1. Click on the code.ros.org/Main repository (in the list on the left pane) to show only ROS packages.
  1. Select all the packages with "boxturtle" in the name, and then install them.
  1. Go back to http://www.ros.org/wiki/ROS/Installation/Ubuntu and perform step 1.4.
  1. Close all terminal windows before moving on to the next step.

# Adding UA Repositories #

  1. Get the rosinstall script:
```
wget --no-check-certificate http://ros.org/rosinstall -O ~/rosinstall
chmod 755 ~/rosinstall
```
  1. Install our ROS repositories (ua-ros-pkg, ua-cs665-ros-pkg) and their dependencies:
```
~/rosinstall -o ~/ros http://ua-ros-pkg.googlecode.com/svn/trunk/config/boxturtle.rosinstall
```
  1. Add this to your bashrc (at the end):
```
source ~/ros/setup.sh
```
  1. Close all terminal windows.
  1. Make all of ua-ros-pkg:
```
~/ros/ua-ros-pkg/make-all.sh
```