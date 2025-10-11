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

#ifndef ISAAC_ROS_E2E_PILOT__PILOT_NET_DECODER_NODE_HPP_
#define ISAAC_ROS_E2E_PILOT__PILOT_NET_DECODER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <string>

namespace nvidia
{
namespace isaac_ros
{
namespace pilot_net
{

class PilotNetDecoderNode : public rclcpp::Node
{
public:
  explicit PilotNetDecoderNode(const rclcpp::NodeOptions & options);

private:
  void tensorCallback(const isaac_ros_tensor_list_interfaces::msg::TensorList::SharedPtr msg);

  // Parameters
  std.string tensor_name_;
  double steer_scale_;
  double speed_scale_;

  // Publisher and Subscriber
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;
  rclcpp::Subscription<isaac_ros_tensor_list_interfaces::msg::TensorList>::SharedPtr tensor_sub_;
};

}  // namespace pilot_net
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_E2E_PILOT__PILOT_NET_DECODER_NODE_HPP_