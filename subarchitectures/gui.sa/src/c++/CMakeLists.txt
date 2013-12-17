# Includes the file that contains the macro cast_slice2cpp
include(Slice2CppBenoit)

set( ROS_CAST_BRIDGE_SA_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../ros_cast_bridge.sa/src )

include_directories(.)
include_directories(${ROS_CAST_BRIDGE_SA_SOURCE_DIR}/c++)

# Compiles to C++ the ICE classes related to the OCU

cast_slice2cpp(GEN_CPP GEN_HPP gui.ice ${CAST_SLICE_DIR}) # This should be removed soon

cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/ocu/msg/cast/DialogueUtteranceMessage.ice ${ROS_CAST_BRIDGE_SA_SOURCE_DIR}/slice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/ocu/msg/cast/SelectionMessage.ice ${ROS_CAST_BRIDGE_SA_SOURCE_DIR}/slice)

# Creates a C++ library with all SLICE files, and links in the CAST library (CDL)
add_library(gui_sa SHARED ${GEN_CPP})
target_link_libraries(gui_sa ros_cast_bridge_sa CDL)

install(TARGETS gui_sa LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
