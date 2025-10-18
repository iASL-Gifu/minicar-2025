# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# (ライセンスヘッダーは適切に設定してください)
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_e2e_traj_control)

# --- 1. 必要なパッケージを検索 ---
# パッケージ作成時に指定した依存関係
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(isaac_ros_nitros REQUIRED)
find_package(isaac_ros_nitros_image_type REQUIRED)
find_package(isaac_ros_nitros_tensor_list_type REQUIRED)
find_package(isaac_ros_common REQUIRED)
find_package(OpenCV REQUIRED) # C++コードがOpenCV APIを直接使うため

# --- 2. ノードのライブラリを作成 ---
add_library(merge_sync_node SHARED
  src/merge_sync_node.cpp
)

target_include_directories(merge_sync_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# --- 3. 依存関係をリンク ---
ament_target_dependencies(merge_sync_node PUBLIC
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "sensor_msgs"
  "nav_msgs"
  "isaac_ros_nitros"
  "isaac_ros_nitros_image_type"
  "isaac_ros_nitros_tensor_list_type"
  "isaac_ros_common"
)

# OpenCVライブラリをリンク
target_link_libraries(merge_sync_node PUBLIC
  ${OpenCV_LIBRARIES}
)

# --- 4. コンポーネントノードとして登録 ---
rclcpp_components_register_nodes(merge_sync_node
  "isaac_ros_e2e_traj_control::MergeSyncNode"
)

# --- 5. インストール設定 ---
install(
  TARGETS merge_sync_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- 6. バージョン情報 ---
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_package()