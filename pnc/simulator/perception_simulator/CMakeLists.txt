cmake_minimum_required(VERSION 3.5)
project(perception_simulator_package)

find_package(ament_cmake_auto REQUIRED)
find_package(glog REQUIRED)
find_package(Protobuf REQUIRED)
find_package(osqp REQUIRED)
find_package(absl REQUIRED)
find_package(perception_em_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
ament_auto_find_build_dependencies()

set(pnc_map_folder ${CMAKE_CURRENT_LIST_DIR}/../../pnc_map)
set(pnc_common_folder ${CMAKE_CURRENT_LIST_DIR}/../../common)
set(simulator_common_folder ${CMAKE_CURRENT_LIST_DIR}/../common)
set(proto_gen_folder ${CMAKE_CURRENT_LIST_DIR}/../../build-special/proto)
file(GLOB PLANNING_FILES
     ${proto_gen_folder}/*.pb.cc
     ${pnc_common_folder}/math/curve_math.cc
     ${pnc_common_folder}/math/vec2d.cc
     ${pnc_common_folder}/math/box2d.cc
     ${pnc_common_folder}/math/polygon2d.cc
     ${pnc_common_folder}/math/line_segment2d.cc
     ${pnc_common_folder}/math/line_segments.cc
     ${pnc_common_folder}/math/math_utils.cc
     ${pnc_common_folder}/file/file.cc
     ${pnc_common_folder}/gflags/global_gflags.cc
     ${pnc_common_folder}/time/time.cc
     ${pnc_common_folder}/third_party/collision_lib/gjk.cc
     ${pnc_common_folder}/util/coordinate.cc
#      ${simulator_common_folder}/scenario_interface/scenario_interface.cc
     ${simulator_common_folder}/hdmap_interface/hdmap_interface.cc
     ${simulator_common_folder}/tinyxml/tinyxml2.cpp
    )

set(INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/../../)
include_directories(
    ${INCLUDE_DIRECTORIES}
    ${proto_gen_folder}
    )

ament_auto_add_executable(perception_simulator_node
               src/main.cpp
               src/object.cpp
               src/perception_node.cpp
               ${PLANNING_FILES}
               )

target_link_libraries(perception_simulator_node
                      glog::glog
                      ${Protobuf_LIBRARIES}
                      osqp::osqp
                      absl::strings
                      )

ament_auto_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../../configs/simulator
        DESTINATION share/${PROJECT_NAME}/configs/)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../../configs/pnc_map
        DESTINATION share/${PROJECT_NAME}/configs/)
