include(Slice2Cpp)

include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to search
cast_slice2cpp(GEN_CPP GEN_HPP planning_roma.ice ../../slice ${CAST_SLICE_DIR})
cast_slice2cpp(GEN_CPP GEN_HPP topograph.ice)
cast_slice2cpp(GEN_CPP GEN_HPP robot_position.ice)
cast_slice2cpp(GEN_CPP GEN_HPP robot_status.ice)
cast_slice2cpp(GEN_CPP GEN_HPP gap.ice)
cast_slice2cpp(GEN_CPP GEN_HPP are.ice)
cast_slice2cpp(GEN_CPP GEN_HPP diagnostic.ice)

add_library(PlanningROMAIce SHARED ${GEN_CPP})
install(TARGETS PlanningROMAIce LIBRARY DESTINATION lib)
