// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_e2e_traj_control/trajcontrolnet_decoder_node.hpp"

#include <cuda_runtime.h>
#include <string>
#include <vector>

namespace isaac_ros_e2e_traj_control
{
TrajcontrolnetDecoderNode::TrajcontrolnetDecoderNode(const rclcpp::NodeOptions options)
: rclcpp::Node("trajcontrolnet_decoder_node", options), 
  nitros_sub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosTensorListView>>(
      this,
      "tensor_input", 
      nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name,
      // コールバックのクラス名を変更
      std::bind(&TrajcontrolnetDecoderNode::InputCallback, this, 
      std::placeholders::_1))},
  pub_path_{create_publisher<nav_msgs::msg::Path>("decoder/path", 10)},
  pub_cmd_{create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("decoder/control_cmd", 10)},
  trajectory_tensor_name_{declare_parameter<std::string>("trajectory_tensor_name", "output_trajectory")},
  commands_tensor_name_{declare_parameter<std::string>("commands_tensor_name", "output_commands")},
  path_frame_id_{declare_parameter<std::string>("path_frame_id", "odom")}
{}

// デストラクタ
TrajcontrolnetDecoderNode::~TrajcontrolnetDecoderNode() = default;

// コールバック関数名を変更
void TrajcontrolnetDecoderNode::InputCallback(const nvidia::isaac_ros::nitros::NitrosTensorListView & msg)
{
  // 1. 2つのテンソルを名前で取得
  auto trajectory_tensor = msg.GetNamedTensor(trajectory_tensor_name_);
  auto commands_tensor = msg.GetNamedTensor(commands_tensor_name_);

  if (trajectory_tensor.GetBuffer() == nullptr || commands_tensor.GetBuffer() == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get one or both tensors by name.");
    return;
  }
  if (trajectory_tensor.GetDataType() != nvidia::isaac_ros::nitros::NitrosDataType::kFloat32 ||
      commands_tensor.GetDataType() != nvidia::isaac_ros::nitros::NitrosDataType::kFloat32)
  {
    RCLCPP_ERROR(this->get_logger(), "Tensor data type is not Float32.");
    return;
  }

  const auto & traj_dims = trajectory_tensor.GetShape().dims;
  const auto & cmd_dims = commands_tensor.GetShape().dims;
  if (traj_dims.size() != 2 || traj_dims[1] != 3 || 
      cmd_dims.size() != 2 || cmd_dims[1] != 2 ||
      traj_dims[0] != cmd_dims[0])
  {
    RCLCPP_ERROR(this->get_logger(), "Unexpected tensor dimensions.");
    return;
  }
  
  const size_t future_len = traj_dims[0]; // 30
  
  // (CPU側にメモリを確保)
  std::vector<float> trajectory_data(future_len * 3); // 30 * 3
  std::vector<float> commands_data(future_len * 2); // 30 * 2

  // (GPU -> CPU へデータをコピー)
  cudaError_t cuda_status;
  cuda_status = cudaMemcpy(
    trajectory_data.data(), trajectory_tensor.GetBuffer(), 
    trajectory_tensor.GetTensorSize(), cudaMemcpyDeviceToHost);
  if (cuda_status != cudaSuccess) {
    RCLCPP_ERROR(this->get_logger(), "Failed to copy trajectory tensor to host: %s", cudaGetErrorString(cuda_status));
    return;
  }
  
  cuda_status = cudaMemcpy(
    commands_data.data(), commands_tensor.GetBuffer(), 
    commands_tensor.GetTensorSize(), cudaMemcpyDeviceToHost);
  if (cuda_status != cudaSuccess) {
    RCLCPP_ERROR(this->get_logger(), "Failed to copy commands tensor to host: %s", cudaGetErrorString(cuda_status));
    return;
  }

  // (ヘッダーを準備)
  std_msgs::msg::Header header;
  header.stamp = msg.GetTimestamp(); 
  header.frame_id = path_frame_id_;

  // (Path メッセージの作成)
  auto path_msg = std::make_unique<nav_msgs::msg::Path>();
  path_msg->header = header;
  path_msg->poses.resize(future_len);

  for (size_t i = 0; i < future_len; ++i) {
    const float x = trajectory_data[i * 3 + 0];
    const float y = trajectory_data[i * 3 + 1];
    path_msg->poses[i].header = header; 
    path_msg->poses[i].pose.position.x = x;
    path_msg->poses[i].pose.position.y = y;
    path_msg->poses[i].pose.position.z = 0.0;
    path_msg->poses[i].pose.orientation.w = 1.0;
  }
  
  // (制御コマンドメッセージの作成)
  auto cmd_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
  cmd_msg->header = header;
  const float steer = commands_data[0 * 2 + 0];
  const float speed = commands_data[0 * 2 + 1];
  cmd_msg->drive.steering_angle = steer;
  cmd_msg->drive.speed = speed;

  // (2つのメッセージをパブリッシュ)
  pub_path_->publish(std::move(path_msg));
  pub_cmd_->publish(std::move(cmd_msg));
}

}  // namespace isaac_ros_e2e_traj_control

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros_e2e_traj_control::TrajcontrolnetDecoderNode)