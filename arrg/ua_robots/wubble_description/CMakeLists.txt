cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_gensrv()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# mesh file generations
rosbuild_find_ros_package(ivcon)
rosbuild_find_ros_package(convex_decomposition)

# iterate through all the stl files to:
#  1. convert to *.iv files
#  2. generate convex decomposed .stl
file(GLOB wubble_stl_files ${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.stl)
set(wubble_gen_files "")

foreach(it ${wubble_stl_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  IF ( ${basename} MATCHES "_convex" )

    message("ignoring stale .._convex.stl file:",${basename})

  ELSE ( ${basename} MATCHES "_convex" )

    # create subdirectory convex
    add_custom_command(
      OUTPUT ${basepath}/convex
      # CMake 2.4 doesn't offer the make_directory command.
      #COMMAND ${CMAKE_COMMAND} -E make_directory
      COMMAND mkdir -p
      ARGS ${basepath}/convex)

    # create subdirectory iv
    add_custom_command(
      OUTPUT ${basepath}/iv
      # CMake 2.4 doesn't offer the make_directory command.
      #COMMAND ${CMAKE_COMMAND} -E make_directory
      COMMAND mkdir -p
      ARGS ${basepath}/iv)

    #create obj files for convex decomposition from stl files
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}.obj
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${it} ${basepath}/convex/${basename}.obj
      DEPENDS ${it} ${basepath}/convex)

    set(wubble_gen_files ${wubble_gen_files} ${basepath}/convex/${basename}.obj)

    #convex decompose object files
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.obj
      COMMAND ${convex_decomposition_PACKAGE_PATH}/convex_decomposition/bin/convex_decomposition
      ARGS ${basepath}/convex/${basename}.obj -v12 -p10
      DEPENDS ${basepath}/convex/${basename}.obj ${basepath}/convex)

    set(wubble_gen_files ${wubble_gen_files} ${basepath}/convex/${basename}_convex.obj)

    #convert obj files back to stlb, put in directory named convex
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.stlb
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stlb
      DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)

    set(wubble_gen_files ${wubble_gen_files} ${basepath}/convex/${basename}_convex.stlb)

    #convert obj files back to stla, put in directory named convex
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.stla
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stla
      DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)

    set(wubble_gen_files ${wubble_gen_files} ${basepath}/convex/${basename}_convex.stla)

    #create iv files
    add_custom_command(
      OUTPUT ${basepath}/iv/${basename}.iv
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${it} ${basepath}/iv/${basename}.iv
      DEPENDS ${basepath}/iv ${it})
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.iv
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.iv
      DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)
     
    set(wubble_gen_files ${wubble_gen_files} ${basepath}/iv/${basename}.iv ${basepath}/convex/${basename}_convex.iv)


  ENDIF ( ${basename} MATCHES "_convex" )

endforeach(it)

add_custom_target(media_files ALL DEPENDS ${wubble_gen_files} )

