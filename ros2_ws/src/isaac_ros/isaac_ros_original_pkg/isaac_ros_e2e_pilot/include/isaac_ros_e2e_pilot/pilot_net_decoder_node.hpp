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

#ifndef ISAAC_ROS_PILOT_NET__PILOT_NET_DEDECODER_NODE_HPP_
#define ISAAC_ROS_PILOT_NET__PILOT_NET_DECODER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "isaac_ros_nitros/nitros_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace pilot_net
{

class PilotNetDecoderNode : public nitros::NitrosNode
{
public:
  explicit PilotNetDecoderNode(const rclcpp::NodeOptions & options);
  ~PilotNetDecoderNode();

  PilotNetDecoderNode(const PilotNetDecoderNode &) = delete;
  PilotNetDecoderNode & operator=(const PilotNetDecoderNode &) = delete;

  // Callback to pass ROS parameters to the GXF graph after it's loaded
  void postLoadGraphCallback() override;

private:
  // ROS Parameters
  std::string tensor_name_;
  double steer_scale_;
  double speed_scale_;
};

}  // namespace pilot_net
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_PILOT_NET__PILOT_NET_DECODER_NODE_HPP_