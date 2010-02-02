cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDeb)
rosbuild_init()

add_definitions(-fPIC)

rosbuild_add_boost_directories()

rosbuild_add_executable(bin/gazebo_ros_diffdrive src/gazebo_ros_diffdrive.cpp)
rosbuild_link_boost(bin/gazebo_ros_diffdrive thread)
