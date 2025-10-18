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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"

// message_filters による同期のためのヘッダ
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

namespace isaac_ros_e2e_traj_control
{

// ROS 2 メッセージ型を使用
using TensorListMsg = isaac_ros_tensor_list_interfaces::msg::TensorList;
using TensorListMsgPtr = TensorListMsg::ConstSharedPtr;

// 同期ポリシーの定義 (ApproximateTimeを使用)
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  TensorListMsg, TensorListMsg>;

class TensorListJoinerNode : public rclcpp::Node
{
public:
  explicit TensorListJoinerNode(const rclcpp::NodeOptions options);

private:
  // 同期コールバック
  void InputCallback(
    const TensorListMsgPtr image_tensor_msg,
    const TensorListMsgPtr odom_tensor_msg);

  // 2つの TensorList を購読
  message_filters::Subscriber<TensorListMsg> sub_image_tensor_;
  message_filters::Subscriber<TensorListMsg> sub_odom_tensor_;

  // 同期クラス
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // 結合された TensorList を発行 (標準のPublisher)
  rclcpp::Publisher<TensorListMsg>::SharedPtr pub_combined_tensor_;
};

}  // namespace isaac_ros_e2e_traj_control