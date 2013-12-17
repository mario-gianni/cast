#include(Slice2Cpp)
include(Slice2CppBenoit)

include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to search
cast_slice2cpp(GEN_CPP GEN_HPP mapping.ice ../../slice ${CAST_SLICE_DIR})
cast_slice2cpp(GEN_CPP GEN_HPP planningControl.ice ../../slice ${CAST_SLICE_DIR})

cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/AreaOfInterest.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/CarObjectOfInterest.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/ElementOfInterest.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/FunctionalArea.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/LocationOfInterest.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/ObjectOfInterest.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/Point3D.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/Polygon.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/Pose.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/SignObjectOfInterest.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/VictimObjectOfInterest.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/Window.ice)
cast_slice2cpp(GEN_CPP GEN_HPP eu/nifti/env/VantagePoint.ice)

add_library(MappingIce SHARED ${GEN_CPP})
install(TARGETS MappingIce LIBRARY DESTINATION lib)

