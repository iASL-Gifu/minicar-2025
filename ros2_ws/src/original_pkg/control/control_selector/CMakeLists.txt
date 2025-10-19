cmake_minimum_required(VERSION 3.8)
project(control_selector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(control_selector_node src/control_selector_node.cpp)
ament_target_dependencies(control_selector_node rclcpp ackermann_msgs std_msgs)
install(TARGETS control_selector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})  

ament_package()

