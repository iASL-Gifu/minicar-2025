// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef NVIDIA_ISAAC_ROS_PILOT_NET_ACKERMANN_MESSAGE_HPP_
#define NVIDIA_ISAAC_ROS_PILOT_NET_ACKERMANN_MESSAGE_HPP_

#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {
namespace isaac_ros {

// A struct to hold AckermannDrive data within a GXF message
struct AckermannDrive {
  float steering_angle;
  float steering_angle_velocity;
  float speed;
  float acceleration;
  float jerk;
};

}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_ROS_PILOT_NET_ACKERMANN_MESSAGE_HPP_