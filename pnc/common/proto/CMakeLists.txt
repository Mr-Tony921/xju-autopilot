cmake_minimum_required(VERSION 3.5)

# set the project name
project(pnc_proto_package)

set(CMAKE_CXX_STANDARD 11)

if (CMAKE_COMPILER_IS_GNUCXX OR MAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++11 -g -Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -pthread -std=c++11")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DENABLE_DEBUG")
set(CMAKE_BUILD_TYPE "Relese")

# find dependencies
# find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)


# copy from FindProtobuf.cmake
function(PROTOBUF_GENERATE_CPP OUTPUT_DIR SRCS HDRS)
    if(NOT ARGN)
        message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP() called without any proto files")
        return()
    endif()
    if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
        # Create an include path for each file specified
        foreach(FIL ${ARGN})
            get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
            get_filename_component(ABS_PATH ${ABS_FIL} PATH)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    else()
        set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    if(DEFINED PROTOBUF_IMPORT_DIRS)
        foreach(DIR ${PROTOBUF_IMPORT_DIRS})
            get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
            list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
            if(${_contains_already} EQUAL -1)
                list(APPEND _protobuf_include_path -I ${ABS_PATH})
            endif()
        endforeach()
    endif()
    set(${SRCS})
    set(${HDRS})
    foreach(FIL ${ARGN})
        get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
        get_filename_component(FIL_WE ${FIL} NAME_WE)
        list(APPEND ${SRCS} "${OUTPUT_DIR}/${FIL_WE}.pb.cc")
        list(APPEND ${HDRS} "${OUTPUT_DIR}/${FIL_WE}.pb.h")
        add_custom_command(
                OUTPUT "${OUTPUT_DIR}/${FIL_WE}.pb.cc"
                "${OUTPUT_DIR}/${FIL_WE}.pb.h"
                COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_DIR}"
                COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
                ARGS --cpp_out  ${OUTPUT_DIR} ${_protobuf_include_path} ${ABS_FIL}
                DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
                COMMENT "Running C++ protocol buffer compiler on ${FIL}"
                VERBATIM )
    endforeach()
    set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
    set(${SRCS} ${${SRCS}} PARENT_SCOPE)
    set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

# By default have PROTOBUF_GENERATE_CPP macro pass -I to protoc
# for each directory where a proto file is referenced.
if(NOT DEFINED PROTOBUF_GENERATE_CPP_APPEND_PATH)
    set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE)
endif()

# place where to generate protobuf sources
set(proto_gen_folder ${CMAKE_CURRENT_LIST_DIR}/../../build-special/proto)
set(proto_lib_folder ${CMAKE_CURRENT_LIST_DIR}/../../build-special/lib)

file(GLOB PNC_PROTO_FILES
     */*.proto
)

# check proto name repeated
set(PROTO_CHECK_FIRST_LIST ${PNC_PROTO_FILES})
set(PROTO_CHECK_SECOND_LIST "")
foreach (proto_check_file1 ${PROTO_CHECK_FIRST_LIST})
    get_filename_component(proto_check_basename1 ${proto_check_file1} NAME)
    foreach (proto_check_file2 ${PROTO_CHECK_SECOND_LIST})
        get_filename_component(proto_check_basename2 ${proto_check_file2} NAME)
        if (${proto_check_basename1} STREQUAL ${proto_check_basename2})
            message(FATAL_ERROR "proto files with the same basename: ${proto_check_file1}, ${proto_check_file2}")
        endif ()
    endforeach()
    list(APPEND PROTO_CHECK_SECOND_LIST ${proto_check_file1})
endforeach()
# check proto name repeated

PROTOBUF_GENERATE_CPP(${proto_gen_folder} PROTO_SRCS PROTO_HDRS "${PNC_PROTO_FILES}")

add_library(pnc_proto STATIC
            ${PROTO_SRCS})

# install(TARGETS pnc_proto DESTINATION ${proto_lib_folder})

# install to default directory using colcon
install(TARGETS pnc_proto 
        DESTINATION
        ARCHIVE DESTINATION lib/pnc_proto
        LIBRARY DESTINATION lib/pnc_proto
        RUNTIME DESTINATION lib/pnc_proto)

message(STATUS "<-------------------->")
message(STATUS "pnc_proto Build SUCCESS!")
message(STATUS "<-------------------->")