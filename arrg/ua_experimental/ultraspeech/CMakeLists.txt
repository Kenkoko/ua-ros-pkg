cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_executable("ros_dvgrab" src/dvgrab-3.5/main.cc src/dvgrab-3.5/dvgrab.cc src/dvgrab-3.5/io.c src/dvgrab-3.5/raw1394util.c src/dvgrab-3.5/avi.cc src/dvgrab-3.5/dvframe.cc src/dvgrab-3.5/error.cc src/dvgrab-3.5/filehandler.cc src/dvgrab-3.5/frame.cc src/dvgrab-3.5/hdvframe.cc src/dvgrab-3.5/iec13818-1.cc src/dvgrab-3.5/iec13818-2.cc src/dvgrab-3.5/ieee1394io.cc src/dvgrab-3.5/riff.cc src/dvgrab-3.5/smiltime.cc src/dvgrab-3.5/srt.cc src/dvgrab-3.5/stringutils.cc src/dvgrab-3.5/v4l2reader.cc) 
rosbuild_add_executable("jack_audio_capture" src/jack_audio_capture.cpp)
rosbuild_add_executable("audio_clip_writer" src/audio_clip_writer.cpp)
rosbuild_add_executable("video_writer" src/collect_video.cpp)
#rosbuild_add_executable(${PROJECT_NAME} src/data_collect.cpp)
#target_link_libraries(${PROJECT_NAME} png)
target_link_libraries("video_writer" png)
target_link_libraries("jack_audio_capture" jack)
target_link_libraries("audio_clip_writer" sndfile)
target_link_libraries("ros_dvgrab" raw1394 avc1394 rom1394 iec61883 dv m quicktime pthread z dl)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
