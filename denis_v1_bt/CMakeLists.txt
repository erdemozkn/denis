cmake_minimum_required(VERSION 3.8)
project(denis_v1_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)


add_library(denis_v1_bt SHARED
  src/check_zone_bt.cpp
)

target_link_libraries(denis_v1_bt
  rclcpp::rclcpp
  behaviortree_cpp::behaviortree_cpp
  tf2_ros::tf2_ros
  yaml-cpp
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS denis_v1_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
