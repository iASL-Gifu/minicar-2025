// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_e2e_traj_control/odom_encoder_node.hpp" 

#include <cuda_runtime.h>
#include <string>
#include <vector>
#include <deque>
#include <cmath>

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_builder.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_builder.hpp"

namespace isaac_ros_e2e_traj_control
{

OdomEncoderNode::OdomEncoderNode(const rclcpp::NodeOptions options)
: rclcpp::Node("odom_encoder_node", options), 
  sub_{create_subscription<nav_msgs::msg::Odometry>(
      "odom_input", 10, std::bind(&OdomEncoderNode::InputCallback, this,
      std::placeholders::_1))},
  nitros_pub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
        nvidia::isaac_ros::nitros::NitrosTensorList>>(
      this, "odom_tensor",
      nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name)}, 
  tensor_name_{declare_parameter<std::string>("tensor_name", "odom_history_tensor")},
  history_size_{static_cast<size_t>(declare_parameter<int>("history_size", 10))} 
{
    RCLCPP_INFO(this->get_logger(), "History size (N) set to: %ld", history_size_);
}

OdomEncoderNode::~OdomEncoderNode() = default;

void OdomEncoderNode::InputCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received odometry message.");

  const auto & pose = msg->pose.pose;
  const auto & twist = msg->twist.twist;

  // 1. 8つの要素を抽出
  std::vector<float> current_odom_vec(8);
  current_odom_vec[0] = static_cast<float>(pose.position.x);
  current_odom_vec[1] = static_cast<float>(pose.position.y);
  current_odom_vec[2] = static_cast<float>(pose.position.z);
  current_odom_vec[3] = static_cast<float>(pose.orientation.x);
  current_odom_vec[4] = static_cast<float>(pose.orientation.y);
  current_odom_vec[5] = static_cast<float>(pose.orientation.z);
  current_odom_vec[6] = static_cast<float>(pose.orientation.w);
  const float vx = static_cast<float>(twist.linear.x);
  const float vy = static_cast<float>(twist.linear.y);
  current_odom_vec[7] = std::sqrt(vx * vx + vy * vy);

  // 2. 履歴に追加
  odom_history_.push_back(current_odom_vec);

  // 3. 履歴が N を超えたら古いものを削除
  if (odom_history_.size() > history_size_) {
    odom_history_.pop_front();
  }

  // 4. 履歴が N フレーム分溜まっていない場合はパブリッシュしない
  if (odom_history_.size() < history_size_) {
    RCLCPP_DEBUG(this->get_logger(), "Buffering odom frames. Current: %ld / Required: %ld",
                 odom_history_.size(), history_size_);
    return;
  }

  // 5. CPU側でデータを平坦化
  const size_t num_elements = history_size_ * 8;
  const size_t buffer_size = num_elements * sizeof(float);
  
  std::vector<float> host_buffer(num_elements);
  size_t idx = 0;
  for (const auto & odom_vec : odom_history_) {
    std::copy(odom_vec.begin(), odom_vec.end(), host_buffer.begin() + idx);
    idx += 8;
  }

  // 6. CUDA バッファを確保
  void * buffer;
  cudaMalloc(&buffer, buffer_size);
  if (buffer == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to allocate CUDA memory (size: %ld bytes).", buffer_size);
    return;
  }

  // 7. CPU から GPU へデータをコピー
  cudaError_t cuda_status = cudaMemcpy(buffer, host_buffer.data(), buffer_size, cudaMemcpyHostToDevice);
  if (cuda_status != cudaSuccess) {
    RCLCPP_ERROR(this->get_logger(), "Failed to copy data to CUDA memory: %s", cudaGetErrorString(cuda_status));
    cudaFree(buffer);
    return;
  }

  // 8. ヘッダー情報
  std_msgs::msg::Header header = msg->header;
  header.frame_id = tensor_name_; 

  // 9. テンソルリストを作成
  nvidia::isaac_ros::nitros::NitrosTensorList tensor_list =
    nvidia::isaac_ros::nitros::NitrosTensorListBuilder()
    .WithHeader(header)
    .AddTensor(
    tensor_name_,
    (
      nvidia::isaac_ros::nitros::NitrosTensorBuilder()
      .WithShape({static_cast<int>(history_size_), 8}) // Shape: [N, 8]
      .WithDataType(nvidia::isaac_ros::nitros::NitrosDataType::kFloat32) // Type: float
      .WithData(buffer)
      .Build()
    )
    )
    .Build();

  RCLCPP_DEBUG(this->get_logger(), "Sending CUDA buffer [N=%ld, 8] with memory at: %p",
               history_size_, buffer);

  // 10. パブリッシュ
  nitros_pub_->publish(tensor_list);
}

}  // namespace isaac_ros_e2e_traj_control

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros_e2e_traj_control::OdomEncoderNode)