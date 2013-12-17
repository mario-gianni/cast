include(Slice2Cpp)

include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to search
cast_slice2cpp(GEN_CPP GEN_HPP navigation.ice ../../slice ${CAST_SLICE_DIR})

add_library(NavigationIce SHARED ${GEN_CPP})
install(TARGETS NavigationIce LIBRARY DESTINATION lib)

