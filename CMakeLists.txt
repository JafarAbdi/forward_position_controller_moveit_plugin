cmake_minimum_required(VERSION 3.22)
project(forward_position_controller_moveit_plugin LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    moveit_ros_control_interface
    std_msgs
    moveit_core
    pluginlib trajectory_msgs moveit_msgs)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

add_library(moveit_ros_control_interface_trajectory_plugin SHARED
            src/forward_position_controller_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_trajectory_plugin
  PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(moveit_ros_control_interface_trajectory_plugin
                           PRIVATE include)
ament_target_dependencies(moveit_ros_control_interface_trajectory_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS moveit_ros_control_interface_trajectory_plugin
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(
  moveit_ros_control_interface moveit_ros_control_interface_plugins.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
