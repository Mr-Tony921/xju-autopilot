cmake_minimum_required(VERSION 3.15)
project(common_topics)

find_package(ament_cmake REQUIRED)
ament_export_include_directories(include)

ament_package()

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

install(DIRECTORY include/
  DESTINATION include)
