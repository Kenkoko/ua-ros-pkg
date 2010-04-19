cmake_minimum_required(VERSION 2.6)

link_directories(/usr/local/lib/SVS)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init ()

rosbuild_add_library(stoc src/stoc.cpp)
target_link_libraries(stoc svscap svs svscalc)
