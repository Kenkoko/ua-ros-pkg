#!/bin/bash

rosinstall ~/ros /opt/ros/electric http://ua-ros-pkg.googlecode.com/svn/trunk/config/electric-deb.rosinstall

echo "source ~/ros/setup.bash" >> ~/.bashrc
echo "export ROBOT=sim" >> ~/.bashrc

source ~/.bashrc

# clean some old cruft that shouldn't even be in a repository
rm -rf `rospack find oldultraspeech`

~/ros/ua-ros-pkg/make-all.sh
echo "Installation Complete..."
