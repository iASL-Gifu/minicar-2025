# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_e2e_traj_control)

# --- 1. 必要なパッケージを検索 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# cv_bridge と sensor_msgs は odom_to_tensor_node では不要
find_package(nav_msgs REQUIRED)
find_package(isaac_ros_nitros REQUIRED)
find_package(isaac_ros_nitros_image_type REQUIRED)
find_package(isaac_ros_nitros_tensor_list_type REQUIRED)
find_package(isaac_ros_common REQUIRED)

# CUDA ランタイムライブラリを見つける (必須)
find_package(CUDA REQUIRED)

# --- 2. ノードのライブラリを作成 ---
# ターゲット名とソースファイルを odom_to_tensor_node に変更
add_library(odom_to_tensor_node SHARED
  src/odom_to_tensor_node.cpp
)

target_include_directories(odom_to_tensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # OpenCV の代わりに CUDA のインクルードパスを追加
  ${CUDA_INCLUDE_DIRS}
)

# --- 3. 依存関係をリンク ---
# ターゲット名を変更
ament_target_dependencies(odom_to_tensor_node PUBLIC
  "rclcpp"
  "rclcpp_components"
  # cv_bridge と sensor_msgs は不要
  "nav_msgs"
  "isaac_ros_nitros"
  "isaac_ros_nitros_image_type"
  "isaac_ros_nitros_tensor_list_type"
  "isaac_ros_common"
)

# OpenCV の代わりに CUDA ランタイムライブラリをリンク
target_link_libraries(odom_to_tensor_node PUBLIC
  CUDA::cudart
)

# --- 4. コンポーネントノードとして登録 ---
# ターゲット名とクラス名を変更 (名前空間に注意)
rclcpp_components_register_nodes(odom_to_tensor_node
  "nvidia::isaac_ros::e2e_traj_control::OdomToTensorNode"
)

# --- 5. インストール設定 ---
install(
  # ターゲット名を変更
  TARGETS odom_to_tensor_node
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