cmake_minimum_required(VERSION 3.5)

project(pnc_configs_package)

find_package(ament_cmake REQUIRED)

# install to default directory using colcon
install(TARGETS ${PROJECT_NAME_NODE}
        DESTINATION
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
