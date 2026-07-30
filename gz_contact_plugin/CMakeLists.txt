cmake_minimum_required(VERSION 3.8)
project(gz_contact_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(contact_plugin SHARED src/contact_plugin.cpp)

target_link_libraries(contact_plugin
  gz-sim8::gz-sim8
)

install(
  TARGETS contact_plugin
  DESTINATION lib
)

ament_package()
