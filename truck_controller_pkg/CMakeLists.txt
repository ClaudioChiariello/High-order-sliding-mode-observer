cmake_minimum_required(VERSION 3.8)
project(truck_controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(control_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

generate_parameter_library(
  truck_controller_parameters
  src/truck_controller_parameters.yaml
)


add_library(truck_controller SHARED src/TruckController.cpp)


target_include_directories(truck_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>)

target_link_libraries(truck_controller
  PUBLIC
    pluginlib::pluginlib
    controller_interface::controller_interface
    rclcpp::rclcpp
    truck_controller_parameters
    ${geometry_msgs_TARGETS})

#ament_target_dependencies(truck_controller
#  rclcpp
#  geometry_msgs
#  controller_interface
#  pluginlib
#  truck_controller_parameters
#)

pluginlib_export_plugin_description_file(controller_interface truck_controller.xml)

install(TARGETS
  truck_controller
  LIBRARY DESTINATION lib
)


install(DIRECTORY
launch
config
urdf
DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
