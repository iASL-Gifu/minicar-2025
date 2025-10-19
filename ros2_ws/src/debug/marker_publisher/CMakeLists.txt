cmake_minimum_required(VERSION 3.8)
project(marker_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(marker_publisher_node src/marker_publisher_node.cpp)
ament_target_dependencies(marker_publisher_node
  rclcpp
  visualization_msgs
  geometry_msgs
)

install(TARGETS marker_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()