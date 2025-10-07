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

#include "isaac_ros_e2e_pilot/pilot_net_decoder_node.hpp" 

#include <string>
#include <vector>
#include <algorithm> 
#include <memory>    

#include <cuda_runtime.h>

#include "rclcpp_components/register_node_macro.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace pilot_net
{

PilotNetDecoderNode::PilotNetDecoderNode(const rclcpp::NodeOptions options)
: rclcpp::Node("pilot_net_decoder_node", options) 
{
  // Initialize the NITROS subscriber
  nitros_sub_ = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosTensorListView>>(
      this,
      "/input/control_tensor",  // Input topic name
      nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name,
      std::bind(&PilotNetDecoderNode::InputCallback, this, std::placeholders::_1));

  // Initialize the Ackermann publisher
  pub_control_ = create_publisher<ackermann_msgs::msg::AckermannDrive>(
      "/output/ackermann_cmd", 10);  // Output topic name

  // Declare and get parameters
  tensor_name_ = declare_parameter<std::string>("tensor_name", "output_tensor");
  steer_scale_ = declare_parameter<double>("steer_scale", 1.0);
  speed_scale_ = declare_parameter<double>("speed_scale", 1.0);

  RCLCPP_INFO(this->get_logger(), "PilotNet Decoder Node has been initialized.");
}

PilotNetDecoderNode::~PilotNetDecoderNode() = default;

void PilotNetDecoderNode::InputCallback(const nvidia::isaac_ros::nitros::NitrosTensorListView & msg)
{
  // 1. Get the tensor from the message by its name
  auto tensor = msg.GetNamedTensor(tensor_name_);

  // Assuming the model's output is two float values: [steer, speed]
  const size_t expected_size_bytes = 2 * sizeof(float);
  if (tensor.GetTensorSize() != expected_size_bytes) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(), "Expected tensor size %zu bytes, but got %zu bytes. Check model output.",
      expected_size_bytes, tensor.GetTensorSize());
    return;
  }

  // 2. Copy the tensor data from GPU memory to a CPU vector
  std::vector<float> control_values(2);
  cudaError_t cuda_status = cudaMemcpy(
    control_values.data(), tensor.GetBuffer(), tensor.GetTensorSize(), cudaMemcpyDeviceToHost);

  if (cuda_status != cudaSuccess) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to copy tensor data from GPU to CPU: %s",
      cudaGetErrorString(cuda_status));
    return;
  }

  // 3. Extract steer and speed values
  float steer = control_values[0];
  float speed = control_values[1];

  // 4. Apply scaling factors from parameters
  float scaled_steer = steer * steer_scale_;
  float scaled_speed = speed * speed_scale_;

  // 5. Clamp the values to the range [-1.0, 1.0]
  float clamped_steer = std::clamp(scaled_steer, -1.0f, 1.0f);
  float clamped_speed = std::clamp(scaled_speed, -1.0f, 1.0f);

  // 6. Create the AckermannDrive message
  auto ackermann_msg = ackermann_msgs::msg::AckermannDrive();
  ackermann_msg.header.stamp = msg.GetTimestamp();
  ackermann_msg.header.frame_id = "base_link"; 
  ackermann_msg.steering_angle = clamped_steer;
  ackermann_msg.speed = clamped_speed;

  // 7. Publish the control message
  pub_control_->publish(ackermann_msg);
}

}  // namespace pilot_net
}  // namespace isaac_ros
}  // namespace nvidia

// Register the node as a component
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::pilot_net::PilotNetDecoderNode)