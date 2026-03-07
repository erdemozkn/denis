cmake_minimum_required(VERSION 3.8)
project(denis_v1_bt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)

# Add your library
add_library(denis_v1_bt SHARED
  src/check_zone_bt.cpp
)

ament_target_dependencies(denis_v1_bt
  rclcpp
  behaviortree_cpp
  geometry_msgs
  tf2
  tf2_ros
  yaml-cpp
  pluginlib
)

target_link_libraries(denis_v1_bt yaml-cpp)

pluginlib_export_plugin_description_file(behaviortree_cpp plugin.xml)

install(TARGETS
  denis_v1_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  geometry_msgs
  tf2
  tf2_ros
  yaml-cpp
  pluginlib
)
ament_export_libraries(denis_v1_bt)

ament_package()