cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(audio_capture)
rosbuild_add_boost_directories()

#uncomment if you have defined services
rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_executable(audio_capture audio_capture.cpp)
target_link_libraries(audio_capture portaudio)

rosbuild_add_executable(audio_clip_writer audio_clip_writer.cpp)
target_link_libraries(audio_clip_writer sndfile)

