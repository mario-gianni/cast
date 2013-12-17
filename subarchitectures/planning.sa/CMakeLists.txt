cmake_minimum_required (VERSION 2.6)
project (Planning)
include(UseCAST)

# auto generated code
add_subdirectory (src/c++/autogen/)
#add_subdirectory (src/python/)

MACRO (add_ros_cast_component _name)
  rosbuild_add_library(${_name} ${ARGN})
  target_link_libraries(${_name} CDL CASTCore CASTArchitecture PlanningIce)
  install(TARGETS ${_name} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
  set(CAST_COMPONENT_NAME ${_name})
ENDMACRO (add_ros_cast_component _name)

include_directories(src/c++/)
include_directories(src/c++/autogen/)
include_directories(include/)
include_directories(.)

include_directories(../mapping.sa/src/c++/autogen)
link_directories(../../../output/lib)

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
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_ros_cast_component(RobotStateReader src/c++/RobotStateReader.cpp)
link_cast_component(${CAST_COMPONENT_NAME} RobotStateReader)
add_ros_cast_component(PlanningBridge src/c++/PlanningBridge.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PlanningBridge)
add_ros_cast_component(DummyPlanningTaskGenerator src/c++/DummyPlanningTaskGenerator.cpp)
link_cast_component(${CAST_COMPONENT_NAME} DummyPlanningTaskGenerator)
add_ros_cast_component(VPPlanner src/c++/VPPlanner.cpp)
link_cast_component(${CAST_COMPONENT_NAME} VPPlanner MappingIce)
