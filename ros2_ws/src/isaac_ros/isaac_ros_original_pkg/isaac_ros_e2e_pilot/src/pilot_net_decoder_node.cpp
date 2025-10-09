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

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

namespace nvidia
{
namespace isaac_ros
{
namespace pilot_net
{

// Configuration for the NITROS pipeline
constexpr char APP_YAML_FILENAME[] = "config/pilot_net_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_e2e_pilot";

// Component and topic keys for mapping (using modern constexpr)
constexpr char INPUT_COMPONENT_KEY[] = "tensor_sub/rx";
constexpr char INPUT_TOPIC_NAME[] = "tensor_sub";

constexpr char OUTPUT_COMPONENT_KEY[] = "ackermann_pub/tx";
constexpr char OUTPUT_TOPIC_NAME[] = "cmd_ackermann";

// C++17-compliant aggregate initialization for the configuration map
const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {INPUT_COMPONENT_KEY,
    {
      nitros::NitrosPublisherSubscriberType::NEGOTIATED,      // .type
      rclcpp::QoS(1),                                         // .qos
      "nitros_tensor_list_nchw_rgb_f32",                      // .compatible_data_format
      INPUT_TOPIC_NAME,                                       // .topic_name
    }
  },
  {OUTPUT_COMPONENT_KEY,
    {
      nitros::NitrosPublisherSubscriberType::NEGOTIATED,      // .type
      rclcpp::QoS(1),                                         // .qos
      "nitros_ackermann_drive",                               // .compatible_data_format
      OUTPUT_TOPIC_NAME,                                      // .topic_name
      INPUT_COMPONENT_KEY,                                    // .frame_id_source_key
    }
  }
};

PilotNetDecoderNode::PilotNetDecoderNode(const rclcpp::NodeOptions & options)
: nitros::NitrosNode(
    options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    {}, {}, {}, // Preset extensions and other configurations
    PACKAGE_NAME)
{
  // Declare and get parameters
  tensor_name_ = declare_parameter<std::string>("tensor_name", "output_tensor");
  steer_scale_ = declare_parameter<double>("steer_scale", 1.0);
  speed_scale_ = declare_parameter<double>("speed_scale", 1.0);

  // Register supported message types
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosTensorList>();
  // You would also register your custom AckermannDrive NITROS type here
  // registerSupportedType<nvidia::isaac_ros::nitros::NitrosAckermannDrive>();

  startNitrosNode();

  RCLCPP_INFO(this->get_logger(), "PilotNet Decoder Node (NitrosNode) has been initialized.");
}

PilotNetDecoderNode::~PilotNetDecoderNode() = default;

void PilotNetDecoderNode::postLoadGraphCallback()
{
  RCLCPP_INFO(get_logger(), "In postLoadGraphCallback(), passing ROS parameters to GXF.");
  // The first argument is the component name in the YAML file.
  // The second is the fully-qualified C++ class name of the GXF component.
  getNitrosContext().setParameterStr(
    "pilotnet_decoder", "nvidia::isaac_ros::PilotNetDecoder", "tensor_name", tensor_name_);
  getNitrosContext().setParameterFloat64(
    "pilotnet_decoder", "nvidia::isaac_ros::PilotNetDecoder", "steer_scale", steer_scale_);
  getNitrosContext().setParameterFloat64(
    "pilotnet_decoder", "nvidia::isaac_ros::PilotNetDecoder", "speed_scale", speed_scale_);
}

}  // namespace pilot_net
}  // namespace isaac_ros
}  // namespace nvidia

// Register the node as a component
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::pilot_net::PilotNetDecoderNode)