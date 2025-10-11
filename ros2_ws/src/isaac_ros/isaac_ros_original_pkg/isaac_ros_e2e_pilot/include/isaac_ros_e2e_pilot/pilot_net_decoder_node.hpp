// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// ... (License Header) ...

#ifndef ISAAC_ROS_E2E_PILOT__PILOT_NET_DECODER_NODE_HPP_
#define ISAAC_ROS_E2E_PILOT__PILOT_NET_DECODER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <string>

namespace nvidia
{
namespace isaac_ros
{
namespace pilot_net
{

class PilotNetDecoderNode : public rclcpp::Node
{
public:
  explicit PilotNetDecoderNode(const rclcpp::NodeOptions & options);

private:
  void tensorCallback(const isaac_ros_tensor_list_interfaces::msg::TensorList::SharedPtr msg);

  // Parameters
  std.string tensor_name_;
  double steer_scale_;
  double speed_scale_;

  // Publisher and Subscriber
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;
  rclcpp::Subscription<isaac_ros_tensor_list_interfaces::msg::TensorList>::SharedPtr tensor_sub_;
};

}  // namespace pilot_net
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_E2E_PILOT__PILOT_NET_DECODER_NODE_HPP_