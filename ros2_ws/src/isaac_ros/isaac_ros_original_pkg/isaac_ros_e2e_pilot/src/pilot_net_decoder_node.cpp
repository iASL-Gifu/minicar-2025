// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// ... (License Header) ...

#include "isaac_ros_e2e_pilot/pilot_net_decoder_node.hpp"
#include <algorithm>
#include "rclcpp_components/register_node_macro.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace pilot_net
{

PilotNetDecoderNode::PilotNetDecoderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pilot_net_decoder_node", options)
{
  // ROSパラメータの宣言と取得
  tensor_name_ = this->declare_parameter<std::string>("tensor_name", "output_tensor");
  steer_scale_ = this->declare_parameter<double>("steer_scale", 1.0);
  speed_scale_ = this->declare_parameter<double>("speed_scale", 1.0);

  // 通常のROS 2 Publisherを作成
  ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
    "/cmd_ackermann", 10);

  // 推論結果のTensorListトピックを購読するSubscriberを作成
  tensor_sub_ = this->create_subscription<isaac_ros_tensor_list_interfaces::msg::TensorList>(
    "/tensor_out", 10,  // 推論ノードの出力トピック名に合わせる
    std::bind(&PilotNetDecoderNode::tensorCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Simple PilotNet Decoder Node has been initialized.");
}

void PilotNetDecoderNode::tensorCallback(
  const isaac_ros_tensor_list_interfaces::msg::TensorList::SharedPtr msg)
{
  // 1. 目的のテンソルを名前で探す
  const auto & tensors = msg->tensors;
  auto it = std::find_if(
    tensors.begin(), tensors.end(),
    [this](const auto & tensor) {
      return tensor.name == tensor_name_;
    });

  if (it == tensors.end()) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Tensor '%s' not found.", tensor_name_.c_str());
    return;
  }
  const auto & target_tensor = *it;

  // 2. テンソルからデータを読み取る (データはCPU上にあります)
  const float * control_outputs = reinterpret_cast<const float *>(target_tensor.data.data());

  // 3. AckermannDriveメッセージを作成して発行
  auto ackermann_msg = std::make_unique<ackermann_msgs::msg::AckermannDrive>();
  ackermann_msg->header = msg->header;  // タイムスタンプを引き継ぐ
  ackermann_msg->steering_angle = control_outputs[0] * steer_scale_;
  ackermann_msg->speed = control_outputs[1] * speed_scale_;
  ackermann_pub_->publish(std::move(ackermann_msg));
}

}  // namespace pilot_net
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::pilot_net::PilotNetDecoderNode)