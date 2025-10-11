// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include "gxf/pilot_net/pilot_net_decoder.hpp"
#include "gxf/pilot_net/ackermann_message.hpp"

#include "gxf/multimedia/tensor.hpp"
#include "gxf/std/timestamp.hpp"
#include "cuda_runtime.h"

namespace nvidia {
namespace isaac_ros {

gxf_result_t PilotNetDecoder::registerInterface(gxf::Registrar* registrar) noexcept {
  gxf::Expected<void> result;
  result &= registrar->parameter(
    tensor_receiver_, "tensor_receiver", "Tensor Input",
    "Receiver to get the tensor from the inference component.");
  result &= registrar->parameter(
    ackermann_transmitter_, "ackermann_transmitter", "Ackermann Output",
    "Transmitter to send the decoded Ackermann command.");
  result &= registrar->parameter(
    tensor_name_, "tensor_name", "Tensor Name",
    "Name of the tensor in the tensor list to be decoded.");
  result &= registrar->parameter(
    steer_scale_, "steer_scale", "Steering Scale",
    "Scaling factor for the steering output.", 1.0);
  result &= registrar->parameter(
    speed_scale_, "speed_scale", "Speed Scale",
    "Scaling factor for the speed output.", 1.0);
  return gxf::ToResultCode(result);
}

gxf_result_t PilotNetDecoder::start() noexcept { return GXF_SUCCESS; }
gxf_result_t PilotNetDecoder::stop() noexcept { return GXF_SUCCESS; }

gxf_result_t PilotNetDecoder::tick() noexcept {
  // 1. 推論コンポーネントからテンソルリストを含むメッセージを受信
  auto maybe_message = tensor_receiver_->receive();
  if (!maybe_message) {
    return gxf::ToResultCode(maybe_message);
  }
  auto message = maybe_message.value();

  // 2. 指定された名前のテンソルをメッセージ内から検索
  auto maybe_tensor = message.get<gxf::Tensor>(tensor_name_.get().c_str());
  if (!maybe_tensor) {
    GXF_LOG_ERROR("Failed to get tensor '%s' from message!", tensor_name_.get().c_str());
    return GXF_FAILURE;
  }
  auto tensor = maybe_tensor.value();

  // 3. テンソルの仕様を検証
  if (tensor->storage_type() != gxf::MemoryStorageType::kDevice) {
    GXF_LOG_ERROR("Tensor must be on GPU (kDevice).");
    return GXF_FAILURE;
  }
  if (tensor->element_type() != gxf::PrimitiveType::kFloat32) {
    GXF_LOG_ERROR("Tensor element type must be float32.");
    return GXF_FAILURE;
  }
  // 想定される出力は [ステアリング, 速度] の2つの値
  if (tensor->rank() != 2 || tensor->shape().dimension(0) != 1 || tensor->shape().dimension(1) != 2) {
    GXF_LOG_ERROR("Tensor shape must be [1, 2] for [steering, speed].");
    return GXF_FAILURE;
  }

  // 4. GPUからCPUへ2つのfloat値をコピー
  float control_outputs[2]; // [steering, speed]
  const cudaError_t cuda_error = cudaMemcpy(
    control_outputs, tensor->pointer(), tensor->size(), cudaMemcpyDeviceToHost);
  if (cuda_error != cudaSuccess) {
    GXF_LOG_ERROR("Failed to copy tensor from device to host: %s", cudaGetErrorString(cuda_error));
    return GXF_FAILURE;
  }

  // 5. 出力用の新しいメッセージエンティティを作成
  auto out_message = gxf::Entity::New(context());
  if (!out_message) { return gxf::ToResultCode(out_message); }

  // 6. AckermannDriveコンポーネントを追加し、デコードした値を設定
  auto ackermann_cmd = out_message.value().add<AckermannDrive>();
  if (!ackermann_cmd) { return gxf::ToResultCode(ackermann_cmd); }
  
  ackermann_cmd.value()->steering_angle = control_outputs[0] * steer_scale_.get();
  ackermann_cmd.value()->speed = control_outputs[1] * speed_scale_.get();
  ackermann_cmd.value()->steering_angle_velocity = 0.0f;
  ackermann_cmd.value()->acceleration = 0.0f;
  ackermann_cmd.value()->jerk = 0.0f;
  
  // 7. 入力メッセージからタイムスタンプをコピー
  auto maybe_timestamp = message.get<gxf::Timestamp>();
  if (maybe_timestamp) {
    auto out_timestamp = out_message.value().add<gxf::Timestamp>();
    *out_timestamp.value() = *maybe_timestamp.value();
  }

  // 8. 最終的なAckermannDriveメッセージを次のコンポーネントへ送信
  return gxf::ToResultCode(ackermann_transmitter_->publish(out_message.value()));
}

}  // namespace isaac_ros
}  // namespace nvidia