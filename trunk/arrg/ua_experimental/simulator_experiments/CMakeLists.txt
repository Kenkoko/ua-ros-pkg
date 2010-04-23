cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()

# GAZEBO PLUGINS

rosbuild_add_library(delete_object_plugin src/delete_object_plugin.cpp)
rosbuild_link_boost(delete_object_plugin thread)

rosbuild_add_library(add_object_plugin src/add_object_plugin.cpp)
rosbuild_link_boost(add_object_plugin thread)

rosbuild_add_library(world_state_publisher src/world_state_publisher.cpp)
rosbuild_link_boost(world_state_publisher thread)

# SIMULATION MANAGER (replacement for main.cc in Gazebo)

rosbuild_add_executable(simulation_manager src/simulation_manager.cpp)
rosbuild_link_boost(simulation_manager thread)
