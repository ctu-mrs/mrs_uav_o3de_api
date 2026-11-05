cmake_minimum_required(VERSION 3.5)
project(mrs_uav_o3de_api)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(DEPENDENCIES
  rclcpp
  pluginlib
  geometry_msgs
  nav_msgs
  sensor_msgs
  mrs_lib
  mrs_msgs
  mrs_uav_hw_api
)

find_package(ament_cmake REQUIRED)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

add_library(MrsUavO3deApi_Plugin SHARED
  src/api.cpp
)

ament_target_dependencies(MrsUavO3deApi_Plugin
  ${DEPENDENCIES}
)

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)

install(
  TARGETS MrsUavO3deApi_Plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
