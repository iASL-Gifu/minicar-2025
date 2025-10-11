# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
project(gxf_isaac_pilot_net)

# 必要なパッケージを検索
find_package(ament_cmake REQUIRED)
find_package(isaac_ros_gxf REQUIRED)

# isaac_ros_gxfが提供するヘルパー関数を使い、GXF Extensionをビルドします。
# これにより、ライブラリの作成、リンク、ヘッダとライブラリのインストールが自動的に行われます。
isaac_ros_gxf_extension(
  NAME pilot_net                                 # 出力されるライブラリ名 (libpilot_net.so)
  SRCS                                            # コンパイルするソースファイル
    gxf/pilot_net/pilot_net_decoder.cpp
    gxf/pilot_net/pilot_net_extension.cpp
  HDRS                                            # インストールするヘッダーファイル
    gxf/pilot_net/ackermann_message.hpp
    gxf/pilot_net/pilot_net_decoder.hpp
  PUBLIC_DEPS                                     # 依存するライブラリ
    isaac_ros_gxf::gxf_core
  INSTALL_HDR_DIR "gxf/pilot_net"                 # ヘッダーをインストールするディレクトリ
)

# isaac_ros_commonを使い、バージョン情報を埋め込みます
find_package(isaac_ros_common REQUIRED)
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

# パッケージの定義を完了
ament_package()