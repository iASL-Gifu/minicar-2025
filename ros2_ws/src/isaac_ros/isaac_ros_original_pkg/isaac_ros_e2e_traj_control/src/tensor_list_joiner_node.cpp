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

#include "isaac_ros_e2e_traj_control/tensor_list_joiner_node.hpp"
#include <memory>
#include <vector>

namespace isaac_ros_e2e_traj_control
{

TensorListJoinerNode::TensorListJoinerNode(const rclcpp::NodeOptions options)
: rclcpp::Node("tensor_list_joiner_node", options),
  // 購読するトピックを初期化 (トピック名は remapping で指定)
  sub_image_tensor_(this, "image_tensor_list"),
  sub_odom_tensor_(this, "odom_tensor_list")
{
  // 出力トピックを初期化 (トピック名は remapping で指定)
  // QoSはTritonノードの入力に合わせる (例: 10)
  pub_combined_tensor_ = this->create_publisher<TensorListMsg>("combined_tensor_list", 10);

  // 同期クラスを初期化 (キューサイズを 10 に設定)
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), sub_image_tensor_, sub_odom_tensor_);
  
  // 同期コールバックを登録
  sync_->registerCallback(
    std::bind(
      &TensorListJoinerNode::InputCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(this->get_logger(), "TensorListJoinerNode initialized.");
}

void TensorListJoinerNode::InputCallback(
  const TensorListMsgPtr image_tensor_msg,
  const TensorListMsgPtr odom_tensor_msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized tensor lists.");

  // 1. 新しい出力メッセージを作成
  auto combined_msg = std::make_unique<TensorListMsg>();

  // 2. ヘッダー情報をコピー (画像側のものを代表として使用)
  //    Tritonノードはタイムスタンプでレイテンシを計算するため、
  //    入力のタイムスタンプを引き継ぐことが重要です。
  combined_msg->header = image_tensor_msg->header;

  // 3. 画像テンソルを新しいリストに追加
  // (std::vector::insert を使って全要素をコピー)
  combined_msg->tensors.insert(
    combined_msg->tensors.end(),
    image_tensor_msg->tensors.begin(),
    image_tensor_msg->tensors.end());

  // 4. オドメトリテンソルを新しいリストに追加
  combined_msg->tensors.insert(
    combined_msg->tensors.end(),
    odom_tensor_msg->tensors.begin(),
    odom_tensor_msg->tensors.end());

  // 5. 結合したメッセージをパブリッシュ
  pub_combined_tensor_->publish(std::move(combined_msg));
}

}  // namespace isaac_ros_e2e_traj_control

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros_e2e_traj_control::TensorListJoinerNode)