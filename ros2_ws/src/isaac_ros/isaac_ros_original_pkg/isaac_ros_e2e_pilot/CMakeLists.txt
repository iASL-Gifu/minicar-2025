# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
project(isaac_ros_e2e_pilot)

# 必要なパッケージを明示的に検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(isaac_ros_tensor_list_interfaces REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(isaac_ros_common REQUIRED)

# ノードのソースコードから共有ライブラリを作成
add_library(pilot_net_decoder_node SHARED
  src/pilot_net_decoder_node.cpp
)

# ライブラリが依存するパッケージをリンク
ament_target_dependencies(pilot_net_decoder_node PUBLIC
  "rclcpp"
  "rclcpp_components"
  "isaac_ros_tensor_list_interfaces"
  "ackermann_msgs"
)

# コンポーネントノードとして登録
rclcpp_components_register_nodes(pilot_net_decoder_node
  "nvidia::isaac_ros::pilot_net::PilotNetDecoderNode"
)

# ヘッダーファイルとライブラリをインストール
install(
  TARGETS pilot_net_decoder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

# launch と config ディレクトリをインストール
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# (もしあれば) スクリプトをインストール
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_model.sh")
  install(PROGRAMS
    scripts/setup_model.sh
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# バージョン情報の埋め込み
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_package()