cmake_minimum_required (VERSION 2.6)
if(POLICY CMP0015)
	cmake_policy(SET CMP0015 OLD)
endif(POLICY CMP0015)


project (mapping.sa)
set( ROSBUILD_DONT_REDEFINE_PROJECT true) # Ensures that the ROS build macros don't change the project name
set( ROS_CAST_BRIDGE_SA_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../ros_cast_bridge.sa/src )
#set( GUI_SA_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../gui.sa/src )
include(UseCAST)

# auto generated code
add_subdirectory(src/c++/autogen/)

MACRO (add_ros_cast_component _name)
    rosbuild_add_library(${_name} ${ARGN})
    target_link_libraries(${_name} CDL CASTCore CASTArchitecture MappingIce)
    install(TARGETS ${_name} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
    set(CAST_COMPONENT_NAME ${_name})
ENDMACRO (add_ros_cast_component _name)

 # This is there because CAST expects all libraries to be in the same folder, so we need to manually link them (because they are located in their respective ROS folders)
MACRO (copy_ROS_lib_to_CAST_folder name)
    add_custom_target(copy_ROS_lib_to_CAST_${name} ALL ln -f -s `rospack find ${name}`/lib/lib${name}.so ../../../output/lib/)
ENDMACRO ()

MACRO (link_ROS_directory name)
    add_custom_target(link_directories(`rospack find ${name}`/lib))
ENDMACRO ()
		  
include_directories(src/c++)
include_directories(src/java)
include_directories(include/)
include_directories(src/c++/autogen/)
include_directories(${ROS_CAST_BRIDGE_SA_SOURCE_DIR}/c++)
include_directories(../planning_roma.sa/src/c++/autogen) 
include_directories(../context.sa/src/c++/autogen) 
link_directories(../../../output/lib)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../subarchitectures/planning_roma.sa/src/c++/autogen/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../subarchitectures/context.sa/src/c++/autogen/)
#set(EXTRA_LD_PATHS ${CMAKE_CURRENT_BINARY_DIR}/../../subarchitectures/planning_roma.sa/src/c++/autogen:${EXTRA_LD_PATHS})
#MESSAGE("I got this path: " ${EXTRA_LD_PATHS})


#include_directories(${GUI_SA_SOURCE_DIR}/c++)
#include_directories(.)
#include(Slice2Cpp)


#Does not work
#set(EXTRA_LD_PATHS ../../stacks/nifti_mapping/eu_nifti_env/lib:${EXTRA_LD_PATHS})
#MESSAGE("I got this path: " ${EXTRA_LD_PATHS})

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_find_ros_package(eu_nifti_env)
link_directories(${eu_nifti_env_PACKAGE_PATH}/lib)

add_ros_cast_component(FEMAreasGenerator src/c++/FEMAreasGenerator.cpp)
link_cast_component(${CAST_COMPONENT_NAME} FEMAreasGenerator eu_nifti_env_msg_ros)
target_link_libraries(FEMAreasGenerator eu_nifti_env)
target_link_libraries(FEMAreasGenerator PlanningROMAIce) 

#add_ros_cast_component(WMReader src/c++/CASTReader.cpp)
#link_cast_component(${CAST_COMPONENT_NAME} WMReader eu_nifti_env)

#add_ros_cast_component(FindFarthestFreeSpacePoint src/c++/findFarthestFreeSpacePoint.cpp)
#target_link_libraries(FindFarthestFreeSpacePoint nifti_contextIce) 

add_ros_cast_component(findForwardUsingPointClouds src/c++/findForwardUsingPointClouds.cpp)
target_link_libraries(findForwardUsingPointClouds nifti_contextIce functional_understanding_of_unstructured_environments functionalMappingHelper) 

add_ros_cast_component(findForwardUsingPointCloudsIterative src/c++/findForwardUsingPointCloudsIterative.cpp)
target_link_libraries(findForwardUsingPointCloudsIterative nifti_contextIce functional_understanding_of_unstructured_environments functionalMappingHelper) 

include_directories(/usr/include/vtk-5.8)
target_link_libraries(findForwardUsingPointClouds ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)


copy_ROS_lib_to_CAST_folder(eu_nifti_env)

