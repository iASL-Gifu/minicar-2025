cmake_minimum_required(VERSION 3.8)
project(bag_manager_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# add executable
add_executable(
  bag_recorder_node
  src/bag_recorder_node.cpp
)

# link libraries
ament_target_dependencies(
  bag_recorder_node
  rclcpp
  std_msgs
  rosbag2_cpp
)

# install rules
install(
  TARGETS bag_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})  


ament_package()