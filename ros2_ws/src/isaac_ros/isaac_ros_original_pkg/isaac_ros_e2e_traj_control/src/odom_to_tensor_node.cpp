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
// WITHOUT WARRANTIES OR CONDITIONS OF ANY, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_e2e_traj_control/odom_to_tensor_node.hpp"

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_builder.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_generic_f32.hpp"

#include <vector>
#include <string>
#include <cmath>     // std::sqrt
#include <algorithm> // std::copy

namespace nvidia
{
namespace isaac_ros
{
namespace e2e_traj_control
{

namespace
{
// CUDAエラーチェック関数 (変更なし)
inline void CheckCudaErrors(cudaError_t code, const char * file, const int line)
{
  if (code != cudaSuccess) {
    const std::string message = "CUDA error returned at " + std::string(file) + ":" +
      std::to_string(line) + ", Error code: " + std::to_string(code) +
      " (" + std::string(cudaGetErrorString(code)) + ")";
    RCLCPP_ERROR(rclcpp::get_logger("OdomToTensorNode"), "%s", message.c_str());
    throw std::runtime_error(message);
  }
}
}  // namespace

OdomToTensorNode::OdomToTensorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("odom_to_tensor_node", options),
  input_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "input_qos")},
  output_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "output_qos")},
  tensor_name_{declare_parameter<std::string>("tensor_name", "odom_history_tensor")},
  history_size_{declare_parameter<int>("history_size", 10)} // N=10をデフォルトに設定
{
  RCLCPP_INFO(
    get_logger(), "Initializing OdomToTensorNode (History Size N=%d, Vector Size=%d)",
    history_size_, kOdomVectorSize);

  if (history_size_ <= 0) {
    RCLCPP_ERROR(get_logger(), "history_size must be > 0.");
    throw std::invalid_argument("history_size must be > 0.");
  }

  // --- バッファの初期化 ---
  // 履歴バッファ(deque)をゼロパディング
  for (int i = 0; i < history_size_; ++i) {
    odom_buffer_.emplace_back(std::vector<float>(kOdomVectorSize, 0.0f));
  }
  // CPU-GPU転送用の一時バッファ(vector)のサイズを確保
  flat_cpu_buffer_.resize(history_size_ * kOdomVectorSize);
  // -------------------------

  // サブスクライバ (標準ROS)
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", input_qos_,
    std::bind(&OdomToTensorNode::OdomCallback, this, std::placeholders::_1));

  // パブリッシャ (NITROS)
  nitros_tensor_pub_ = std::make_shared<
    nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosTensorList>>(
    this, "tensor",
    nvidia::isaac_ros::nitros::nitros_tensor_list_generic_f32_t::supported_type_name,
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig{}, output_qos_);

  // CUDAストリームの作成
  CheckCudaErrors(cudaStreamCreate(&stream_), __FILE__, __LINE__);
}

OdomToTensorNode::~OdomToTensorNode()
{
  RCLCPP_INFO(get_logger(), "Destroying OdomToTensorNode");
  CheckCudaErrors(cudaStreamDestroy(stream_), __FILE__, __LINE__);
}

void OdomToTensorNode::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // 1. CPU側で現在の8次元ベクトルを作成
  std::vector<float> current_odom_vec(kOdomVectorSize);

  const auto & pose = msg->pose.pose;
  const auto & twist = msg->twist.twist;

  // pos(3)
  current_odom_vec[0] = static_cast<float>(pose.position.x);
  current_odom_vec[1] = static_cast<float>(pose.position.y);
  current_odom_vec[2] = static_cast<float>(pose.position.z);
  // ori(4)
  current_odom_vec[3] = static_cast<float>(pose.orientation.x);
  current_odom_vec[4] = static_cast<float>(pose.orientation.y);
  current_odom_vec[5] = static_cast<float>(pose.orientation.z);
  current_odom_vec[6] = static_cast<float>(pose.orientation.w);
  // speed(1)
  const float vx = static_cast<float>(twist.linear.x);
  const float vy = static_cast<float>(twist.linear.y);
  current_odom_vec[7] = std::sqrt(vx * vx + vy * vy);

  // 2. 履歴バッファ (deque) を更新
  odom_buffer_.pop_front();         // 一番古いデータを削除
  odom_buffer_.push_back(current_odom_vec); // 最新のデータを追加

  // 3. GPU転送用にデータを平坦化 (Flatten)
  //    (deque<vector> から 1つの contiguousな vector へコピー)
  size_t idx = 0;
  for (const auto & vec : odom_buffer_) {
    // vec (8要素) を flat_cpu_buffer_ の適切な位置にコピー
    std::copy(vec.begin(), vec.end(), flat_cpu_buffer_.begin() + idx);
    idx += kOdomVectorSize;
  }
  
  // 4. GPU側に必要なメモリを確保
  float * gpu_buffer{nullptr};
  const size_t buffer_size = flat_cpu_buffer_.size() * sizeof(float); // N * 8 * sizeof(float)
  CheckCudaErrors(
    cudaMallocAsync(&gpu_buffer, buffer_size, stream_), __FILE__, __LINE__);

  // 5. CPUからGPUへデータを非同期コピー
  CheckCudaErrors(
    cudaMemcpyAsync(
      gpu_buffer, flat_cpu_buffer_.data(), buffer_size,
      cudaMemcpyHostToDevice, stream_), __FILE__, __LINE__);

  // 6. NITROSテンソルリストを構築 (形状: [N, 8])
  nvidia::isaac_ros::nitros::NitrosTensorList tensor_list =
    nvidia::isaac_ros::nitros::NitrosTensorListBuilder()
    .WithHeader(msg->header) // odomのヘッダー情報を引き継ぐ
    .AddTensor(
    tensor_name_, (nvidia::isaac_ros::nitros::NitrosTensorBuilder()
    .WithShape({static_cast<int32_t>(history_size_), kOdomVectorSize}) // 形状 [N, 8]
    .WithDataType(nvidia::isaac_ros::nitros::NitrosDataType::kFloat32)
    .WithData(gpu_buffer) // GPUポインタを渡す
    .Build()))
    .Build();

  // 7. パブリッシュ
  nitros_tensor_pub_->publish(tensor_list);

  // 8. GPU操作の同期
  CheckCudaErrors(cudaStreamSynchronize(stream_), __FILE__, __LINE__);
}

}  // namespace e2e_traj_control
}  // namespace isaac_ros
}  // namespace nvidia

#include "rclcpp_components/register_node_macro.hpp"
// 登録するクラスの名前空間をパッケージ名に合わせる 
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::e2e_traj_control::OdomToTensorNode)