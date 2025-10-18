// SPDX-FileCopyrightText: 2024 NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_E2E_TRAJ_CONTROL__ODOM_TO_TENSOR_NODE_HPP_
#define ISAAC_ROS_E2E_TRAJ_CONTROL__ODOM_TO_TENSOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_builder.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "isaac_ros_common/qos.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"

#include <cuda_runtime.h>
#include <string>
#include <vector>
#include <deque>   // 履歴バッファ用
#include <cmath>   // std::sqrt用

namespace nvidia
{
namespace isaac_ros
{
namespace e2e_traj_control
{

// Pythonロジックに基づく8次元ベクトル
constexpr int kOdomVectorSize = 8;

class OdomToTensorNode : public rclcpp::Node
{
public:
  explicit OdomToTensorNode(const rclcpp::NodeOptions & options);
  ~OdomToTensorNode();

private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // 標準ROS (CPU) からの入力
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  // NITROS (GPU) への出力
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
    nvidia::isaac_ros::nitros::NitrosTensorList>> nitros_tensor_pub_;

  // QoS設定
  const rclcpp::QoS input_qos_;
  const rclcpp::QoS output_qos_;

  // CUDA操作用のストリーム
  cudaStream_t stream_;

  // --- 履歴バッファ関連 ---
  std::deque<std::vector<float>> odom_buffer_;
  std::vector<float> flat_cpu_buffer_;

  // パラメータ
  std::string tensor_name_;
  int history_size_;  // 履歴サイズ (N)
};

}  // namespace e2e_traj_control
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_E2E_TRAJ_CONTROL__ODOM_TO_TENSOR_NODE_HPP_