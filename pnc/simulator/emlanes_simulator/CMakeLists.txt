cmake_minimum_required(VERSION 3.5)
project(emlanes_simulator_package)

find_package(ament_cmake_auto REQUIRED)
find_package(glog REQUIRED)
find_package(Protobuf REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(perception_em_msgs REQUIRED)
ament_auto_find_build_dependencies()

set(proto_gen_folder ${CMAKE_CURRENT_LIST_DIR}/../../build-special/proto)
set(pnc_common_folder ${CMAKE_CURRENT_LIST_DIR}/../../common)
set(simulator_common_folder ${CMAKE_CURRENT_LIST_DIR}/../common)
file(GLOB PLANNING_FILES
     ${proto_gen_folder}/*.pb.cc
     ${pnc_common_folder}/file/file.cc
     ${pnc_common_folder}/gflags/global_gflags.cc
     ${pnc_common_folder}/math/vec2d.cc
     ${pnc_common_folder}/math/math_utils.cc
     ${pnc_common_folder}/math/line_segments.cc
     ${pnc_common_folder}/math/line_segment2d.cc
     ${pnc_common_folder}/math/curve_math.cc
     ${pnc_common_folder}/util/coordinate.cc
     ${pnc_common_folder}/time/time.cc
#      ${simulator_common_folder}/scenario_interface/scenario_interface.cc
     ${simulator_common_folder}/hdmap_interface/hdmap_interface.cc
     ${simulator_common_folder}/tinyxml/tinyxml2.cpp
    )

set(INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/../../)

include_directories(
    ${INCLUDE_DIRECTORIES}
    ${proto_gen_folder})

ament_auto_add_executable(emlanes_simulator_node
                          src/main.cpp
                          src/hdmap_server.cpp
                          ${PLANNING_FILES}
                          )

target_link_libraries(emlanes_simulator_node
                      ${Protobuf_LIBRARIES}
                      glog::glog
                      )

ament_auto_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../../configs/
        DESTINATION share/${PROJECT_NAME}/configs/)
