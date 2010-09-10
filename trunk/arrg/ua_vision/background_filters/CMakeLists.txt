cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()

rosbuild_add_library(background_filters src/floor_filter.cpp)
rosbuild_add_executable(floor_filter_node src/floor_filter_node.cpp)
target_link_libraries(floor_filter_node background_filters)

#rosbuild_add_executable(ground_filter src/ground_filter.cpp)

# NEW
rosbuild_add_executable(background_averager src/background_averager.cpp)

# Background subtractor
rosbuild_add_library(background_subtractor src/background_subtractor.cpp src/lbp_background_subtractor.cpp)

rosbuild_add_executable(background_subtractor_node src/background_subtractor_node.cpp)
target_link_libraries(background_subtractor_node background_subtractor)

rosbuild_add_executable(lbp_background_subtractor_node src/lbp_background_subtractor_node.cpp)
target_link_libraries(lbp_background_subtractor_node background_subtractor)

rosbuild_add_executable(shape_finder src/shape_finder.cpp)

# OLD
#rosbuild_add_executable(overhead_filter src/overhead_filter.cpp)

# BAD
#rosbuild_add_executable(background_sub_rewrite src/background_sub_rewrite.cpp)
#rosbuild_add_executable(object_tracker src/object_tracker.cpp)
