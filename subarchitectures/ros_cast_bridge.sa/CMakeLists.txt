cmake_minimum_required (VERSION 2.6)
project (ros_cast_bridge.sa)
set( ROSBUILD_DONT_REDEFINE_PROJECT true) # Ensures that the ROS build macros don't change the project name

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

include_directories(src/c++)
add_subdirectory(src/c++)
