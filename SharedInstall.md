# Introduction #

Instructions for **system administrators** to setup up a shared installation on a class machines.

# Steps #

  1. Switch to the root user
```
sudo su -
```

  1. Download the install script
```
wget http://ua-ros-pkg.googlecode.com/svn/trunk/config/install_ros_shared
```

  1. Make it executable
```
chmod +x install_ros_shared
```

  1. Run it
```
./install_ros_shared
```

The script will download and build a lot of ROS packages, which will take up to an hour (it can be left unsupervised once it's done asking for permission to install ubuntu packages). There may be error messages in the output, but these are typical and not important.