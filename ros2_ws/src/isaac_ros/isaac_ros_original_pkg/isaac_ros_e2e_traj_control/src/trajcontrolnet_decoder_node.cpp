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
      nvidia::isaac_ros::nitros::nitros_tensor_list_nhwc_rgb_f32_t::supported_type_name,
      std::bind(&TrajcontrolnetDecoderNode::InputCallback, this,
      std::placeholders::_1))},
  pub_path_{create_publisher<nav_msgs::msg::Path>("~/path", 10)},
  pub_cmd_{create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("~/control_cmd", 10)},
  trajectory_tensor_name_{declare_parameter<std::string>("trajectory_tensor_name", "output_trajectory")},
  commands_tensor_name_{declare_parameter<std::string>("commands_tensor_name", "output_commands")},
  path_frame_id_{declare_parameter<std::string>("path_frame_id", "odom")}
{}

TrajcontrolnetDecoderNode::~TrajcontrolnetDecoderNode() = default;

void TrajcontrolnetDecoderNode::InputCallback(const nvidia::isaac_ros::nitros::NitrosTensorListView & msg)
{
  auto trajectory_tensor = msg.GetNamedTensor(trajectory_tensor_name_);
  auto commands_tensor = msg.GetNamedTensor(commands_tensor_name_);

  if (trajectory_tensor.GetBuffer() == nullptr || commands_tensor.GetBuffer() == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get one or both tensors by name. Check Triton config.pbtxt output names.");
    return;
  }

  const auto & traj_shape = trajectory_tensor.GetShape().shape();
  const auto & cmd_shape = commands_tensor.GetShape().shape();

  // 形状の妥当性チェック
  // 軌跡 (Trajectory) が [1, 30, 3] であることを期待
  if (traj_shape.rank() != 3 || traj_shape.dimension(0) != 1 ||
      traj_shape.dimension(1) != 30 || traj_shape.dimension(2) != 3)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Unexpected trajectory tensor dimensions. "
      "Got [Rank %u, Dims(%d, %d, %d)] vs Expected [Rank 3, Dims(1, 30, 3)].",
      traj_shape.rank(),
      traj_shape.rank() > 0 ? traj_shape.dimension(0) : -1,
      traj_shape.rank() > 1 ? traj_shape.dimension(1) : -1,
      traj_shape.rank() > 2 ? traj_shape.dimension(2) : -1);
    return;
  }

  // コマンド (Commands) が [1, 30, 2] であることを期待
  if (cmd_shape.rank() != 3 || cmd_shape.dimension(0) != 1 ||
      cmd_shape.dimension(1) != 30 || cmd_shape.dimension(2) != 2)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Unexpected command tensor dimensions. "
      "Got [Rank %u, Dims(%d, %d, %d)] vs Expected [Rank 3, Dims(1, 30, 2)].",
      cmd_shape.rank(),
      cmd_shape.rank() > 0 ? cmd_shape.dimension(0) : -1,
      cmd_shape.rank() > 1 ? cmd_shape.dimension(1) : -1,
      cmd_shape.rank() > 2 ? cmd_shape.dimension(2) : -1);
    return;
  }

  const size_t future_len = traj_shape.dimension(1); // 30
  const size_t cmd_timesteps = cmd_shape.dimension(1); // 30


  std::vector<float> trajectory_data(future_len * 3);
  std::vector<float> commands_data(cmd_timesteps * 2); // 30 * 2 で正しい

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

  std_msgs::msg::Header header;
  header.stamp.sec = msg.GetTimestampSeconds();
  header.stamp.nanosec = msg.GetTimestampNanoseconds();
  header.frame_id = path_frame_id_;

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

  auto cmd_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
  cmd_msg->header = header;

  // 30フレーム分のコマンドのうち、先頭(0番目)のデータを使用する
  const float steer = commands_data[0 * 2 + 0];
  const float speed = commands_data[0 * 2 + 1];

  cmd_msg->drive.steering_angle = steer;
  cmd_msg->drive.speed = speed;

  pub_path_->publish(std::move(path_msg));
  pub_cmd_->publish(std::move(cmd_msg));
}

}  // namespace isaac_ros_e2e_traj_control

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros_e2e_traj_control::TrajcontrolnetDecoderNode)