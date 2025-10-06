cmake_minimum_required(VERSION 3.8)
project(control_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(control_filter_node src/control_filter_node.cpp)
ament_target_dependencies(control_filter_node rclcpp ackermann_msgs)
install(TARGETS control_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})  

ament_package()

