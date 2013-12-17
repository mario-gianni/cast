# original copied from ORCA: http://orca-robotics.sourceforge.net/
# hacked around a bit to reflect a different structure... make that a lot!






# This file contains a set of macros used for generating 
# C++ source from Slice files
#
# Author: Alex Brooks
#

#
# Appends the new_bit to the original.
# If the original is not set, it will be set to the new_bit.
#
MACRO( APPEND original new_bit )

  IF    ( NOT ${original} )
    SET( ${original} ${new_bit} )
  ELSE  ( NOT ${original} )
    SET( ${original} ${${original}} ${new_bit} )
  ENDIF( NOT ${original} )

ENDMACRO( APPEND original new_bit )


# Generate rules for SLICE->C++ files generation, for each
# of the named slice source files.
#
# cast_slice2cpp( generated_cpp_list generated_header_list SRC [INCDIR1 [INCDIR2...]] )
# 
# Returns lists of all the .cpp and .h files that will be generated.
#   (ie modifies the first two arguments: generated_cpp_list and generated_header_list)
#
# Sets up dependencies: 
#  - Each .cpp and .hpp file depends on all the .ice files.
#
# Each optional extra dir is added to the search path for includes

MACRO( cast_slice2cpp generated_cpp_list generated_header_list src_name)

    SET( slice_cpp_suffixes        .cpp )
    SET( slice_header_suffixes     .hpp  )
    SET( slice_suffixes            ${slice_cpp_suffixes} ${slice_header_suffixes} )
    SET( slice2cpp_command         ${ICE_HOME}/bin/slice2cpp${EXE_EXTENSION} )
    
    SET( slice_src_dir        	   src/slice)


    FOREACH(slice_inc_dir ${ARGN})
    	APPEND(slice_include_dirs "-I${slice_inc_dir}")
    ENDFOREACH(slice_inc_dir ${ARGN})

    APPEND(slice_include_dirs "-I${PROJECT_SOURCE_DIR}/src/slice")

    #MESSAGE("slice_include_dirs:" ${slice_include_dirs})

    # This allows to generate the files in the same folder hierarchy as the .ice file is
    # Removes the last slash and whatever is after
    string(REGEX REPLACE "/[^/]*$" "" folder_hierarchy "${src_name}")

    # If there was no folder hierarchy given, then nothing was replaced. We can set the folder hierarchy to the current directory
    if("${folder_hierarchy}" STREQUAL "${src_name}")
        set(folder_hierarchy ".")
    else()
        # It is necessary to include this directory because ICE generates the .cpp files with #include directives without the path (i.e. <#include myHeader.h>)
        # If multiple files from the same directory are compiled, then this will get called multiple times. It's redundant, but not a major problem.
        include_directories( ${folder_hierarchy} ) 
    endif()

    # We need to ensure that the folder exists, before writing a file in it.
    if( NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${folder_hierarchy} )
        #MESSAGE( "Creating ${CMAKE_CURRENT_SOURCE_DIR}/${folder_hierarchy} )" )
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${folder_hierarchy} )
    endif()

    # satellite projects need to include slice files from orca installation
    # NOTE: funky interaction between cmake and slice2cpp: cannot use "" around the slice_args!
    SET( slice_args ${SLICE_PROJECT_ARGS} ${slice_include_dirs} --stream --header-ext hpp --output-dir ${CMAKE_CURRENT_SOURCE_DIR}/${folder_hierarchy} )

    APPEND(slice_src_full "${PROJECT_SOURCE_DIR}/${slice_src_dir}/${src_name}" )
    #MESSAGE( "slice_src_full ${PROJECT_SOURCE_DIR}/${slice_src_dir}/${src_name}" )


    
        SET( slice_source "${PROJECT_SOURCE_DIR}/${slice_src_dir}/${src_name}" )
        #	             MESSAGE( "DEBUG: Dealing with ${src_name}")
    

        #
        # Add a custom cmake command to generate each type of output file: headers and source
        #
        FOREACH ( suffix ${slice_suffixes} )
    
            # OUTPUT is the target we're generating rules for.
            STRING( REGEX REPLACE "\\.ice" ${suffix} output_basename "${src_name}" )
            SET( output_fullname "${CMAKE_CURRENT_SOURCE_DIR}/${output_basename}" )

           #message( "DEBUG output_fullname ${output_fullname}" )

                
            #
            # Add this output to the list of generated files
            #
            IF( ${suffix} STREQUAL ".cpp" )
                APPEND( ${generated_cpp_list} ${output_fullname} )
                SET( generated_file_type "source" )
            ELSE( ${suffix} STREQUAL ".cpp" )
                APPEND( ${generated_header_list} ${output_fullname} )
                SET( generated_file_type "header" )
            ENDIF( ${suffix} STREQUAL ".cpp" )
        

            #
            # Add the command to generate file.xxx from file.ice
            # Note: when the 'output' is needed, the 'command' will be called with the 'args'
            #
            #MESSAGE( STATUS "DEBUG: Adding rule for generating ${output_basename} from ${src_name}" )
            ADD_CUSTOM_COMMAND(
                OUTPUT  ${output_fullname}
                COMMAND ${slice2cpp_command} ${slice_args} ${slice_source}
                DEPENDS ${slice_src_full}
		COMMENT "Generating ${generated_file_type} file from ${slice_source}"
                VERBATIM
            )
    
        ENDFOREACH ( suffix ${slice_suffixes} )
    

    
   
#     MESSAGE( STATUS "DEBUG: generated_cpp_list: ${${generated_cpp_list}}")
 #      MESSAGE( STATUS "DEBUG: generated_header_list: ${${generated_header_list}}")
    
    MESSAGE( STATUS "Will generate cpp header and source files from Slice definition using this command:" )
    MESSAGE( STATUS "${slice2cpp_command} ${slice_args}" )

ENDMACRO( cast_slice2cpp generated_cpp_list generated_header_list )
