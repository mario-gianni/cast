include(Slice2Cpp)

include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to search
cast_slice2cpp(GEN_CPP GEN_HPP planning.ice ../../slice ${CAST_SLICE_DIR})

add_library(PlanningIce SHARED ${GEN_CPP})
install(TARGETS PlanningIce LIBRARY DESTINATION lib)

