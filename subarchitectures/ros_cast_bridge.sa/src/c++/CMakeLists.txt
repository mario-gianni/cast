# Includes the file that contains the macro cast_slice2cpp
include(Slice2CppBenoit)

set( GUI_SA_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../gui.sa/src )
set( MAPPING_SA_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../mapping.sa/src )

include_directories(.)
include_directories(${GUI_SA_SOURCE_DIR}/c++)
include_directories(${MAPPING_SA_SOURCE_DIR}/c++/autogen)

# Compiles to C++ the ICE classes related to bridging between ros and CAST

#MESSAGE( ${ROS_BRIDGE_SA_SLICE_DIR} )
#MESSAGE( ${CAST_SLICE_DIR} )

cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/RequestForROSTime.ice)

cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/ros/Header.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/ros/Time.ice)

# Creates a C++ library with all SLICE files, and links in the CAST library (CDL)
add_library(ros_cast_bridge_sa SHARED ${GEN_CPP})
target_link_libraries(ros_cast_bridge_sa CDL)

install(TARGETS ros_cast_bridge_sa LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

# Creates a CAST component and links the ROS library (to be able to call ros::init)
add_cast_component(NIFTiCentralServer eu/nifti/NIFTiCentralServer.cpp)
link_cast_component(NIFTiCentralServer ros_cast_bridge_sa roscpp)

# Creates a CAST component and links the ROS library (to be able to call ros::init)
add_cast_component(ROS_CAST_Converter eu/nifti/ROS_CAST_Converter.cpp)
link_cast_component(ROS_CAST_Converter ros_cast_bridge_sa roscpp gui_sa)
