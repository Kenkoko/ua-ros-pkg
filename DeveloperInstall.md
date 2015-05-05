
```
sudo apt-get install build-essential python-yaml cmake subversion git-core mercurial python-setuptools
```
  1. Install the rosinstall program
```
sudo easy_install -U rosinstall
```
  1. Install ROS!
```
rosinstall ~/ros /opt/ros/electric http://ua-ros-pkg.googlecode.com/svn/trunk/config/electric-deb.rosinstall
```
  1. Update your .bashrc
```
echo "source ~/ros/setup.bash" >> ~/.bashrc
echo "export ROBOT=sim" >> ~/.bashrc
source ~/.bashrc
```
  1. Run the script that makes all the UA packages
```
~/ros/ua-ros-pkg/make-all.sh
```