include(Slice2Cpp)
include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to search
cast_slice2cpp(GEN_CPP GEN_HPP context.ice ../../slice ${CAST_SLICE_DIR})
cast_slice2cpp(GEN_CPP GEN_HPP forwardRequest.ice ../../slice ${CAST_SLICE_DIR})
cast_slice2cpp(GEN_CPP GEN_HPP forwardResponse.ice ../../slice ${CAST_SLICE_DIR})

add_library(nifti_contextIce SHARED ${GEN_CPP})
install(TARGETS nifti_contextIce LIBRARY DESTINATION lib)
