cmake_minimum_required(VERSION 3.5)
project(debug_tool_package)

find_package(ament_cmake_auto REQUIRED)
find_package(glog REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(planning_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(perception_em_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})

ament_auto_find_build_dependencies()

set(proto_gen_folder ${CMAKE_CURRENT_LIST_DIR}/../../build-special/proto)
set(pnc_common_folder ${CMAKE_CURRENT_LIST_DIR}/../../common)
file(GLOB PLANNING_FILES
     ${proto_gen_folder}/*.pb.cc
     ${pnc_common_folder}/file/file.cc
     ${pnc_common_folder}/math/box2d.cc
     ${pnc_common_folder}/math/math_utils.cc
     ${pnc_common_folder}/math/polygon2d.cc
     ${pnc_common_folder}/math/vec2d.cc
     ${pnc_common_folder}/gflags/global_gflags.cc
     ${pnc_common_folder}/vehicle_config/vehicle_config_provider.cc
     ${pnc_common_folder}/time/time.cc     
     ${pnc_common_folder}/third_party/collision_lib/gjk.cc
    )

set(INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/../../)

include_directories(
    ${INCLUDE_DIRECTORIES}
    ${proto_gen_folder})

ament_auto_add_executable(planning_debug_tool_node
                          planning_debug/planning_main.cpp
                          planning_debug/planning_debug_tool.cpp
                          ${PLANNING_FILES}
                          )

target_link_libraries(planning_debug_tool_node
                      ${Protobuf_LIBRARIES}
                      glog::glog
                      python3.10
                      )

ament_auto_package()

install(DIRECTORY launch meshes DESTINATION share/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../../configs/
        DESTINATION share/${PROJECT_NAME}/configs/)
