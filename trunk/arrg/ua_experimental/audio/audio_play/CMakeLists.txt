cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(audio_play)
#rosbuild_add_boost_directories()

rosbuild_add_executable(audio_play audio_play.cpp)
target_link_libraries(audio_play portaudio)

