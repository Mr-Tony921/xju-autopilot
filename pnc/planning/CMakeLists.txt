cmake_minimum_required(VERSION 3.5)

project(planning_package)


set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR MAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -std=c++17")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DENABLE_DEBUG")
set(CMAKE_BUILD_TYPE "Relese")

find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(absl REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options)

# interface
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(chart_msgs REQUIRED)
find_package(chassis_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(localization_msgs REQUIRED)
find_package(perception_em_msgs REQUIRED)
find_package(planning_msgs REQUIRED)
find_package(routing_msgs REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

add_definitions("-DVERSION=\"${PROJECT_NAME}.${GIT_COMMIT_BRANCH}.${GIT_COMMIT_HASH}\"")

set(proto_gen_folder ${CMAKE_CURRENT_LIST_DIR}/../build-special/proto)
set(proto_lib_folder ${CMAKE_CURRENT_LIST_DIR}/../build-special/lib)
set(pnc_common_folder ${CMAKE_CURRENT_LIST_DIR}/../common)
set(pnc_map_folder ${CMAKE_CURRENT_LIST_DIR}/../pnc_map)

file(GLOB PLANNING_FILES
     ${CMAKE_CURRENT_LIST_DIR}/*.c
     ${CMAKE_CURRENT_LIST_DIR}/*.cc
     ${CMAKE_CURRENT_LIST_DIR}/*/*.c
     ${CMAKE_CURRENT_LIST_DIR}/*/*.cc
     ${CMAKE_CURRENT_LIST_DIR}/*/*/*.c
     ${CMAKE_CURRENT_LIST_DIR}/*/*/*.cc
     ${CMAKE_CURRENT_LIST_DIR}/*/*/*/*.c
     ${CMAKE_CURRENT_LIST_DIR}/*/*/*/*.cc
     ${CMAKE_CURRENT_LIST_DIR}/*/*/*/*/*.c
     ${CMAKE_CURRENT_LIST_DIR}/*/*/*/*/*.cc
     ${proto_gen_folder}/*.pb.cc
     ${pnc_common_folder}/*.c
     ${pnc_common_folder}/*.cc
     ${pnc_common_folder}/*/*.c
     ${pnc_common_folder}/*/*.cc
     ${pnc_common_folder}/*/*/*.c
     ${pnc_common_folder}/*/*/*.cc
     ${pnc_common_folder}/*/*/*/*.cc
     ${pnc_common_folder}/*/*/*/*/*.cc
     ${pnc_map_folder}/*.c
     ${pnc_map_folder}/*.cc
     ${pnc_map_folder}/*/*.c
     ${pnc_map_folder}/*/*.cc)

LIST(FILTER PLANNING_FILES EXCLUDE REGEX ".+_test[.]+h")  #remove *_test.h
LIST(FILTER PLANNING_FILES EXCLUDE REGEX ".+_test[.]+cc") #remove *_test.cc
LIST(FILTER PLANNING_FILES EXCLUDE REGEX "main[.]+cc") #remove *_test.cc

set(INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/../)

include_directories(
    ${INCLUDE_DIRECTORIES}
    ${proto_gen_folder})

include_directories(${Boost_INCLUDE_DIRS})

# add_library(${PROJECT_NAME} SHARED
#             ${PLANNING_FILES})

set(PROJECT_NAME_NODE planning_node)
add_executable(${PROJECT_NAME_NODE} interface/ros/main.cc
               ${PLANNING_FILES})

target_link_libraries(${PROJECT_NAME_NODE} gflags
                      Eigen3::Eigen
                      ${Boost_LIBRARIES}
                      osqp::osqp
                      glog::glog
                      ${Protobuf_LIBRARIES}
                      hpipm
                      blasfeo
                      absl::strings
                      )

# install configs
# set(PLANNIG_PACKAGE_SAVED_FOLDER ${CMAKE_CURRENT_LIST_DIR}/../output)
# install(TARGETS ${PROJECT_NAME_NODE} DESTINATION ${PLANNIG_PACKAGE_SAVED_FOLDER}/node)
# install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../configs/planning DESTINATION lib/${PLANNIG_PACKAGE_SAVED_FOLDER}/configs)

# install to default directory using colcon
install(TARGETS ${PROJECT_NAME_NODE}
        DESTINATION
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
        ${CMAKE_CURRENT_LIST_DIR}/interface/ros/launch 
        DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../configs/
        DESTINATION share/${PROJECT_NAME}/configs/)

ament_target_dependencies(${PROJECT_NAME_NODE} 
                          rclcpp 
                          geometry_msgs 
                          visualization_msgs 
                          common_msgs
                          chart_msgs
                          chassis_msgs
                          control_msgs
                          localization_msgs
                          perception_em_msgs
                          planning_msgs
                          routing_msgs)
ament_package()
 
# find_package(GTest REQUIRED)

# LIST(FILTER PLANNING_FILES EXCLUDE REGEX "main.cc") 
# add_executable(planning_test 
#               #  common/speed/speed_data_test.cc 
#               #  common/speed/st_boundary_test.cc 
#               #  scenarios/common/lane_decider_test/lane_decider_test.cc 
#               #  tasks/deciders/path_bound_decider/path_bound_decider_test.cc 
#               #  tasks/deciders/lateral_shift_decider/lateral_shift_decider_test.cc 
#               #  tasks/deciders/path_decider/path_decider_test/path_decider_test.cc 
#         #       tasks/deciders/st_graph_decider/speed_limit_decider_test.cc
#         #        tasks/optimizers/speed_optimizer/piecewise_jerk_speed_optimizer_test.cc
#               #  tasks/optimizers/speed_optimizer/speed_optimizer_test.cc
#                tasks/optimizers/path_optimizer/path_optimizer_file_test.cc
#                 # ../common/thread_pool/thread_pool_test.cc
#                ${PLANNING_FILES})
# target_link_libraries(planning_test 
#                       gflags
#                       glog::glog
#                       Eigen3::Eigen
#                       ${Boost_LIBRARIES}
#                       osqp::osqp
#                       ${PROTOBUF_LIBRARIES}
#                       GTest::GTest
#                       GTest::Main
#                       ${GTEST_MAIN_LIBRARIES}
#                       hpipm
#                       blasfeo
#                       absl::strings)
# ament_target_dependencies(planning_test rclcpp geometry_msgs 
#                           visualization_msgs
#                           planning_msgs
#                           perception_em_msgs
#                           localization_msgs
#                           control_msgs
#                           common_msgs
#                           chassis_msgs
#                           chart_msgs
#                           routing_msgs) 
