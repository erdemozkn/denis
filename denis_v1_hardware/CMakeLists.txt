cmake_minimum_required(VERSION 3.8)
project(denis_v1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/diff_denis_system.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
serial
)

pluginlib_export_plugin_description_file(
  hardware_interface
  denis_v1_hardware.xml
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES denis_v1_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
