#---------------------------------------------------------------------------
# Top-level NIFTi project CMake file. Inspired by the build file for the
# CogX project by Nick Hawes, Marc Hanheide and others.
#---------------------------------------------------------------------------
# Note that this file may potentially be *shared* by others via
# svn:externals!
#---------------------------------------------------------------------------

cmake_minimum_required (VERSION 2.6)
project (NIFTi)
#>
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

MARK_AS_ADVANCED(CMAKE_BACKWARDS_COMPATIBILITY)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)
MARK_AS_ADVANCED(CMAKE_BUILD_TYPE)
#<

get_filename_component(SYSTEM_ROOT . ABSOLUTE)

# set the output directory.
set(OUTPUT ${SYSTEM_ROOT}/output CACHE PATH "Path prefix for installation")
set(CMAKE_INSTALL_PREFIX "${OUTPUT}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)

# use CMake files packaged with CAST as well
set(CMAKE_MODULE_PATH /usr/share/cast/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH /usr/local/share/cast/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH /opt/local/share/cast/cmake ${CMAKE_MODULE_PATH})

# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH "${SYSTEM_ROOT}/cmake;${CAST_CMAKE_DIR}" ${CMAKE_MODULE_PATH})

#>
SET(PREFIX ${CMAKE_INSTALL_PREFIX})
SET(BINDIR "${PREFIX}/bin")
#<

#---------------------------------------------------------------------------
# Find packages and paths
#---------------------------------------------------------------------------

find_package (CAST REQUIRED)
include (UseCAST)
include (CASTBuild)

# look for ICE_SLICE_DIR that contains Ice/Identity.ice
find_file (SLICE_IDENTITY Ice/Identity.ice
	PATHS /usr/share/slice /usr/share/Ice/slice /opt/Ice/slice)
if (SLICE_IDENTITY)
	string (REGEX REPLACE "/Ice/Identity\\.ice$" "" ICE_SLICE_DIR ${SLICE_IDENTITY})
endif (SLICE_IDENTITY)

#>
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cast-client-start.in ${CMAKE_CURRENT_BINARY_DIR}/cast-client-start)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cast-server-start.in ${CMAKE_CURRENT_BINARY_DIR}/cast-server-start)
INSTALL(PROGRAMS 
	${CMAKE_CURRENT_BINARY_DIR}/cast-client-start 
	${CMAKE_CURRENT_BINARY_DIR}/cast-server-start 
	DESTINATION ${BINDIR})
#<


#---------------------------------------------------------------------------
# C++ code build config
#---------------------------------------------------------------------------

add_definitions (-Wall)

if (USE_CCACHE)
	set (CMAKE_CXX_COMPILER "ccache")
	set (CMAKE_CXX_COMPILER_ARG1 "g++")
	set (CMAKE_C_COMPILER "ccache")
	set (CMAKE_C_COMPILER_ARG1 "gcc")
endif (USE_CCACHE)

#---------------------------------------------------------------------------
# Java code build config
#---------------------------------------------------------------------------

configure_file (${CMAKE_SOURCE_DIR}/cmake/build.properties.in ${CMAKE_SOURCE_DIR}/build.properties)

option (DO_ANT "Automatically call Ant" YES)
if (DO_ANT)
	ADD_CUSTOM_TARGET (ant_build ALL)
		ADD_CUSTOM_COMMAND (
			TARGET  ant_build
			COMMAND ant -Dcastslice.dir=${CAST_SLICE_DIR} -Dcast.jar=${CAST_INSTALL_ROOT}/share/java/cast.jar -Dice.jar=${ICE_HOME}/share/java/Ice.jar WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
endif (DO_ANT)

#---------------------------------------------------------------------------
# OPTIONAL BUILDS: what to build and what not to build
#---------------------------------------------------------------------------

# shared SLICE defs ----------------------------------------------------

# if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/shared-slice AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/shared-slice)
#	add_subdirectory (subarchitectures/shared-slice)
set (BUILD_DEFS_SLICE ON CACHE BOOL "Build slice")
# endif (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/shared-slice AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/shared-slice)

# CAST subarchitectures ----------------------------------------------------

if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/planning.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/planning.sa)
	option (BUILD_SA_PLANNING "Build planning subarchitecture" NO)
	if (BUILD_SA_PLANNING)
		add_subdirectory (subarchitectures/planning.sa)
	endif (BUILD_SA_PLANNING)
endif (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/planning.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/planning.sa)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

 if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/planning_roma.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/planning_roma.sa)
	option (BUILD_SA_PLANNING_ROMA "Build planning_roma subarchitecture" NO)
	if (BUILD_SA_PLANNING_ROMA)
		add_subdirectory (subarchitectures/planning_roma.sa)
	endif (BUILD_SA_PLANNING_ROMA)
endif (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/planning_roma.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/planning_roma.sa)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/dialogue.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/dialogue.sa)
	option (BUILD_SA_DIALOGUE "Build dialogue subarchitecture" NO)
	if (BUILD_SA_DIALOGUE)
		add_subdirectory (subarchitectures/dialogue.sa)
	endif (BUILD_SA_DIALOGUE)
endif (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/dialogue.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/dialogue.sa)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/context.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/context.sa)
	option (BUILD_SA_CONTEXT "Build context subarchitecture" NO)
	if (BUILD_SA_CONTEXT)
		add_subdirectory (subarchitectures/context.sa)
	endif (BUILD_SA_CONTEXT)
endif (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/context.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/context.sa)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/memory.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/memory.sa)
	option (BUILD_SA_MEMORY "Build memory subarchitecture" NO)
	if (BUILD_SA_MEMORY)
		add_subdirectory (subarchitectures/memory.sa)
	endif (BUILD_SA_MEMORY)
endif (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/memory.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/memory.sa)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/mapping.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/mapping.sa)
	option (BUILD_SA_MAPPING "Build mapping subarchitecure" NO)
	if (BUILD_SA_MAPPING)
		add_subdirectory (subarchitectures/mapping.sa)
	endif (BUILD_SA_MAPPING)
endif (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/mapping.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/mapping.sa)

if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/navigation.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/navigation.sa)
	option (BUILD_SA_NAVIGATION "Build navigation subarchitecture" NO)
	if (BUILD_SA_NAVIGATION)
		add_subdirectory (subarchitectures/navigation.sa)
	endif (BUILD_SA_NAVIGATION)
endif (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/navigation.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/navigation.sa)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/gui.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/gui.sa)
	option (BUILD_SA_GUI "Build GUI subarchitecture" NO)
	if (BUILD_SA_GUI)
		add_subdirectory (subarchitectures/gui.sa)
	endif (BUILD_SA_GUI)
endif (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/gui.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/gui.sa)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/subarchitectures/ros_cast_bridge.sa AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/subarchitectures/ros_cast_bridge.sa)
	option (BUILD_SA_ROS_CAST_BRIDGE "Build ROS_CAST_BRIDGE subarchitecture" NO)
	if (BUILD_SA_ROS_CAST_BRIDGE)
		add_subdirectory (subarchitectures/ros_cast_bridge.sa)
	endif ()
endif ()

# CAST tools ---------------------------------------------------------------

# FIXME: ugly! define a macro for testing whether the directory exists

if (EXISTS ${CMAKE_SOURCE_DIR}/tools/math AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/math)
	option (BUILD_TOOLS_MATH "Build math" NO)
	if (BUILD_TOOLS_MATH)
		add_subdirectory (tools/math)
	endif (BUILD_TOOLS_MATH)
endif (EXISTS ${CMAKE_SOURCE_DIR}/tools/math AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/math)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/tools/castutils AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/castutils)
	option (BUILD_TOOLS_CASTUTILS "Build CASTUtils" NO)
#	if (BUILD_TOOLS_CASTUTILS)
#		add_subdirectory (tools/castutils)
#	endif (BUILD_TOOLS_CASTUTILS)
endif (EXISTS ${CMAKE_SOURCE_DIR}/tools/castutils AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/castutils)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/tools/beliefs AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/beliefs)
	option (BUILD_TOOLS_BELIEFS "Build beliefs" NO)
	if (BUILD_TOOLS_BELIEFS)
		add_subdirectory (tools/beliefs/apis/beliefs)
		add_subdirectory (tools/beliefs/apis/beliefs+cast)
	endif (BUILD_TOOLS_BELIEFS)
endif (EXISTS ${CMAKE_SOURCE_DIR}/tools/beliefs AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/beliefs)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/tools/elm AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/elm)
	option (BUILD_TOOLS_ELM "Build ELM" NO)
	if (BUILD_TOOLS_ELM)
		add_subdirectory (tools/elm)
	endif (BUILD_TOOLS_ELM)
endif (EXISTS ${CMAKE_SOURCE_DIR}/tools/elm AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/elm)

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

if (EXISTS ${CMAKE_SOURCE_DIR}/tools/abducer AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/abducer)
	option (BUILD_ABDUCER "Build abducer" NO)
	if (BUILD_ABDUCER)
		add_subdirectory (tools/abducer)
	endif (BUILD_ABDUCER)
endif (EXISTS ${CMAKE_SOURCE_DIR}/tools/abducer AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/abducer)

# ROS stacks ---------------------------------------------------------------
#
#include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#
#if (EXISTS ${CMAKE_SOURCE_DIR}/stacks/nifti_mapping AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/stacks/nifti_mapping)
#	option (BUILD_STK_MAPPING "Build mapping stack" NO)
#	if (BUILD_STK_MAPPING)
#		add_subdirectory (stacks/nifti_mapping)
#	endif (BUILD_STK_MAPPING)
#endif (EXISTS ${CMAKE_SOURCE_DIR}/stacks/nifti_mapping AND IS_DIRECTORY ${CMAKE_SOURCE_DIR}/stacks/nifti_mapping)
