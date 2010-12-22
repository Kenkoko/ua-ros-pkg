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

#rosbuild_genmsg()
#rosbuild_gensrv()

rosbuild_add_boost_directories()
include_directories(src)

rosbuild_add_library (wubble_mdp
					  src/wubble_mdp/entity.cpp
                      src/wubble_mdp/robot.cpp
                      src/wubble_mdp/location.cpp
                      src/wubble_mdp/object.cpp
                      src/wubble_mdp/item.cpp
                      src/wubble_mdp/relations.cpp
                      src/wubble_mdp/environment.cpp
                     )

rosbuild_add_executable(environment src/environment_node.cpp)
target_link_libraries(environment wubble_mdp)

