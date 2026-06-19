cmake_minimum_required(VERSION 3.8)
project(truck_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(truck_control_base INTERFACE)
target_include_directories(truck_control_base INTERFACE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

#target_link_libraries(truck_control_base INTERFACE
#rclcpp::rclcpp
#${geometry_msgs_LIBRARIES}
#${std_msgs_LIBRARIES}
#)

add_executable(truck_kinematic_control src/truck_kinematic_control.cpp)
target_link_libraries(truck_kinematic_control truck_control_base)
ament_target_dependencies(truck_kinematic_control rclcpp geometry_msgs std_msgs)

install(TARGETS
truck_kinematic_control
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
launch
config
urdf
DESTINATION share/${PROJECT_NAME}
)

install(
DIRECTORY include/
DESTINATION include
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
