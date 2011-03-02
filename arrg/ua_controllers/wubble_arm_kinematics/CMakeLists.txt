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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_library(wubble_arm_kinematics_lib
  src/wubble_arm_ik.cpp
  src/wubble_arm_kinematics_utils.cpp
  src/wubble_arm_ik_solver.cpp
  src/wubble_arm_kinematics.cpp
)

rosbuild_add_executable(wubble_arm_kinematics_node src/main.cpp)
target_link_libraries(wubble_arm_kinematics_node wubble_arm_kinematics_lib)

rosbuild_add_executable(wubble_arm_ik_gen_f0 include/wubble_arm_kinematics/wubble_arm_ik_gen_f0.cpp)
rosbuild_add_executable(wubble_arm_ik_gen_f1 include/wubble_arm_kinematics/wubble_arm_ik_gen_f1.cpp)
rosbuild_add_executable(wubble_arm_ik_gen_f2 include/wubble_arm_kinematics/wubble_arm_ik_gen_f2.cpp)

rosbuild_add_executable(wubble_ik_single_test src/test/test_single_ik.cpp)

#target_link_libraries(example ${PROJECT_NAME})
