cmake_minimum_required (VERSION 2.6)
project (nifti_context)
include(UseCAST)

# auto generated code
add_subdirectory(src/c++/autogen)

include_directories(src/c++/autogen)

link_directories(../../../output/lib)

#include_directories(src/c++)
#add_subdirectory(src/c++)
