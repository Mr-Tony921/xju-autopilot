cmake_minimum_required(VERSION 3.5)

project(rviz_xju_plugins)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rviz_default_plugins REQUIRED)
find_package(visualization_msgs REQUIRED)

set(rviz_xju_plugins_headers_to_moc
  include/goal_tool.hpp
  include/pose_tool.hpp
)

foreach(header "${rviz_xju_plugins_headers_to_moc}")
  qt5_wrap_cpp(rviz_xju_plugins_moc_files "${header}")
endforeach()

set(rviz_xju_plugins_source_files
  src/goal_tool.cpp
  src/pose_tool.cpp
)

add_library(rviz_xju_plugins SHARED
  ${rviz_xju_plugins_moc_files}
  ${rviz_xju_plugins_source_files}
)

target_include_directories(rviz_xju_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(rviz_xju_plugins PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_xju_plugins PRIVATE "RVIZ_XJU_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(rviz_xju_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rviz_xju_plugins
  PUBLIC
  geometry_msgs
  rclcpp
  rviz_common
  rviz_rendering
  # rviz_default_plugins
  visualization_msgs
)

ament_export_include_directories(include)
ament_export_targets(rviz_xju_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rviz_common
  geometry_msgs
  rclcpp
  rviz_ogre_vendor
  # rviz_default_plugins
  visualization_msgs
)

install(
  TARGETS rviz_xju_plugins
  EXPORT rviz_xju_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
#   DESTINATION "share/${PROJECT_NAME}"
# )

ament_package(
  CONFIG_EXTRAS "rviz_xju_plugins-extras.cmake"
)
