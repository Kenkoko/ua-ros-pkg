cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

rosbuild_add_boost_directories()

rosbuild_add_library(background_filters src/floor_filter.cpp)

rosbuild_add_executable(floor_filter_node src/floor_filter_node.cpp)
target_link_libraries(floor_filter_node background_filters)

rosbuild_add_executable(overhead_filter src/overhead_filter.cpp)
rosbuild_add_executable(background_sampler src/background_sampler.cpp)
