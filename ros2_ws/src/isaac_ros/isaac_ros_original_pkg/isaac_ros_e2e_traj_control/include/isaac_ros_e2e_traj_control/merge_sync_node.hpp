#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_view.hpp" // <-- Image View から変更
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "isaac_ros_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_common/qos.hpp"

#include <deque>
#include <mutex>
#include <array>
#include <vector>
#include <cuda_runtime.h>

namespace isaac_ros_e2e_traj_control
{

class MergeSyncNode : public rclcpp::Node
{
public:
  explicit MergeSyncNode(const rclcpp::NodeOptions & options);
  ~MergeSyncNode();

private:
  // --- ROS 2 インターフェース ---
  // NitrosTensorList を購読
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
    nvidia::isaac_ros::nitros::NitrosTensorListView>> nitros_tensor_sub_;
    
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
    nvidia::isaac_ros::nitros::NitrosTensorList>> nitros_tensor_pub_;

  // --- コールバック ---
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // NitrosTensorListView を受け取る
  void TensorCallback(const nvidia::isaac_ros::nitros::NitrosTensorListView & tensor_list_view);

  // --- 状態 (オドメトリバッファ) ---
  std::mutex buffer_mutex_;
  std::deque<std::array<float, 8>> odom_buffer_;

  // --- パラメータ ---
  size_t past_len_{5};
  size_t odom_dim_{8};
  std::string image_tensor_name_in_list_; // 入力TensorList内の画像テンソル名
  std::string image_tensor_name_in_model_; // モデル(ONNX)が期待する画像テンソル名
  std::string odom_tensor_name_in_model_;  // モデル(ONNX)が期待するOdomテンソル名

  // --- CUDA 関連 ---
  cudaStream_t stream_{nullptr};
  float* odom_cpu_pinned_buffer_{nullptr};
  float* odom_gpu_buffer_{nullptr};
};

}  // namespace isaac_ros_e2e_traj_control