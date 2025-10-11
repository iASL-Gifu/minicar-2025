// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVIDIA_ISAAC_ROS_PILOT_NET_DECODER_HPP_
#define NVIDIA_ISAAC_ROS_PILOT_NET_DECODER_HPP_

#include <string>
#include "gxf/core/gxf.h"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace isaac_ros {

// Decodes a tensor into an AckermannDrive command
class PilotNetDecoder : public gxf::Codelet {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) noexcept override;
  gxf_result_t start() noexcept override;
  gxf_result_t tick() noexcept override;
  gxf_result_t stop() noexcept override;

 private:
  // Input for the tensor list
  gxf::Parameter<gxf::Handle<gxf::Receiver>> tensor_receiver_;
  // Output for the Ackermann command
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> ackermann_transmitter_;

  // Parameters to be set from the ROS wrapper node
  gxf::Parameter<std::string> tensor_name_;
  gxf::Parameter<double> steer_scale_;
  gxf::Parameter<double> speed_scale_;
};

}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_ROS_PILOT_NET_DECODER_HPP_