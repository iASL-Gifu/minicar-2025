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


#include <deque>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "isaac_ros_nitros/nitros_publisher.hpp"
#include "isaac_ros_nitros/types/nitros_tensor_list.hpp"

namespace isaac_ros_e2e_traj_control 
{

class OdomEncoderNode : public rclcpp::Node
{
public:
  explicit OdomEncoderNode(const rclcpp::NodeOptions options);
  ~OdomEncoderNode();

private:
  void InputCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosTensorList>> nitros_pub_;

  std::string tensor_name_;
  size_t history_size_; 
  
  std::deque<std::vector<float>> odom_history_;
};

}  // namespace isaac_ros_e2e_traj_control