cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_classpath("${wubble_world_2d_PACKAGE_PATH}/lib")
message("Java build classpath is ${_java_classpath}")

add_runtime_classpath("${wubble_world_2d_PACKAGE_PATH}/lib")
message("Java runtime classpath is ${_java_runtime_classpath}")

add_jar_dir("${wubble_world_2d_PACKAGE_PATH}/lib")
message("Java build classpath is ${_java_classpath}")
message("Java runtime classpath is ${_java_runtime_classpath}")

add_ld_lib_path("${wubble_world_2d_PACKAGE_PATH}/natives/linux")
message("Java LD_LIBRARY path is ${_ld_lib_path}")

# TODO: Copy resource files from src to bin, not just java classes


# Add source & executables
add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
rospack_add_java_executable(main edu.arizona.simulator.ww2d.Main) 
