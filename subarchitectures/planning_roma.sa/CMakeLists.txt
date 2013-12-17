cmake_minimum_required (VERSION 2.6)

if(POLICY CMP0015)
	cmake_policy(SET CMP0015 OLD)
endif(POLICY CMP0015)

project (PlanningROMA)
include(UseCAST)

# auto generated code
add_subdirectory (src/c++/autogen/)
#add_subdirectory (src/prolog)

MACRO (add_ros_cast_component _name)
  rosbuild_add_library(${_name} ${ARGN})
  target_link_libraries(${_name} CDL CASTCore CASTArchitecture PlanningROMAIce)
  install(TARGETS ${_name} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
  set(CAST_COMPONENT_NAME ${_name})
ENDMACRO (add_ros_cast_component _name)

include_directories(src/c++/)
include_directories(src/c++/autogen/)
include_directories(include/)
include_directories(.)
include_directories($ENV{ECLIPSE_INCLUDE})

include_directories(../mapping.sa/src/c++/autogen)
include_directories(../context.sa/src/c++/autogen)
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

link_directories($ENV{ECLIPSE_LIB})

# ROS_CAST Component which reads from the ROS topic the current region in which the robot is
#add_ros_cast_component(POSNAVWriter src/c++/POSNAVWriter.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} POSNAVWriter)

# ROS_CAST Component which reads from the ROS topic the online 2D segmentation of the environment
#add_ros_cast_component(TopoGraphWriter src/c++/TopoGraphWriter.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} TopoGraphWriter)

# ROS_CAST Component which reads from the ROS topic the status of the battery and of the flippers
#add_ros_cast_component(RobotStatusWriter src/c++/RobotStatusWriter.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} RobotStatusWriter)

# ROS_CAST Component which 
# 1) reads from the WMs the representation of environment
# 2) generates plans
# 3) executes the plans 

rosbuild_add_boost_directories()

add_ros_cast_component(ExecutionMonitoring src/c++/ExecutionMonitoring.cpp src/c++/EmbeddedEclipse.cpp src/c++/PlanningUtils.cpp src/c++/Parser.cpp)
link_cast_component(${CAST_COMPONENT_NAME} ExecutionMonitoring)
target_link_libraries(ExecutionMonitoring eclipse MappingIce)
rosbuild_link_boost(ExecutionMonitoring thread)

add_ros_cast_component(TaskWriter src/c++/TaskWriter.cpp)
link_cast_component(${CAST_COMPONENT_NAME} TaskWriter)

add_ros_cast_component(ActionFailureManager src/c++/ActionFailureManager.cpp)
link_cast_component(${CAST_COMPONENT_NAME} ActionFailureManager)

add_ros_cast_component(NavigationComponent src/c++/NavigationComponent.cpp)
link_cast_component(${CAST_COMPONENT_NAME} NavigationComponent nifti_contextIce)

#add_ros_cast_component(PoseWriter src/c++/PoseWriter.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} PoseWriter)

add_ros_cast_component(LocomotionComponent src/c++/LocomotionComponent.cpp)
link_cast_component(${CAST_COMPONENT_NAME} LocomotionComponent)

add_ros_cast_component(LidarComponent src/c++/LidarComponent.cpp)
link_cast_component(${CAST_COMPONENT_NAME} LidarComponent)

add_ros_cast_component(TopoSegBuilderComponent src/c++/TopoSegBuilderComponent.cpp)
link_cast_component(${CAST_COMPONENT_NAME} TopoSegBuilderComponent)

add_ros_cast_component(TopoSegReaderComponent src/c++/TopoSegReaderComponent.cpp)
link_cast_component(${CAST_COMPONENT_NAME} TopoSegReaderComponent)

add_ros_cast_component(GapDetectionComponent src/c++/GapDetectionComponent.cpp)
link_cast_component(${CAST_COMPONENT_NAME} GapDetectionComponent)

add_ros_cast_component(ArmComponent src/c++/ArmComponent.cpp)
link_cast_component(${CAST_COMPONENT_NAME} ArmComponent)

add_ros_cast_component(AREInterfaceComponent src/c++/AREInterfaceComponent.cpp)
link_cast_component(${CAST_COMPONENT_NAME} AREInterfaceComponent)

add_ros_cast_component(DiagnosticComponent src/c++/DiagnosticComponent.cpp)
link_cast_component(${CAST_COMPONENT_NAME} DiagnosticComponent)
