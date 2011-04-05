cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# Add dependencies
rosbuild_find_ros_package(time_series)
rosbuild_find_ros_package(verb_learning)
rosbuild_find_ros_package(wubble_world_2d)
message("Found libs at ${time_series_PACKAGE_PATH}/lib")

add_classpath("${time_series_PACKAGE_PATH}/bin") # This seems like a hack
add_classpath("${wubble_world_2d_PACKAGE_PATH}/bin") # This seems like a hack
add_classpath("${time_series_PACKAGE_PATH}/lib")
add_classpath("${verb_learning_PACKAGE_PATH}/lib")
add_classpath("${wubble_world_2d_PACKAGE_PATH}/wubble_svn/lib")
message("Java build classpath is ${_java_classpath}")

add_runtime_classpath("${wubble_world_2d_PACKAGE_PATH}/bin") # Need this for the "data" in ww2d
add_runtime_classpath("${time_series_PACKAGE_PATH}/lib")
add_runtime_classpath("${verb_learning_PACKAGE_PATH}/lib")
add_runtime_classpath("${wubble_world_2d_PACKAGE_PATH}/wubble_svn/lib")
message("Java runtime classpath is ${_java_runtime_classpath}")

add_jar_dir("${time_series_PACKAGE_PATH}/lib")
add_jar_dir("${verb_learning_PACKAGE_PATH}/lib")
add_jar_dir("${wubble_world_2d_PACKAGE_PATH}/wubble_svn/lib")
message("Java build classpath is ${_java_classpath}")
message("Java runtime classpath is ${_java_runtime_classpath}")

add_ld_lib_path("${wubble_world_2d_PACKAGE_PATH}/wubble_svn/natives/linux")
message("Java LD_LIBRARY path is ${_ld_lib_path}")

# Add source & executables
add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
#add_java_source_dir(${wubble_world_2d_PACKAGE_PATH}/src)
rospack_add_java_executable(interface edu.arizona.verbs.main.Interface) 
