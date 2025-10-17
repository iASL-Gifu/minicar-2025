#include "trajectory_preprocessor/merge_sync_node.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_builder.hpp"

namespace trajectory_preprocessor
{

// CUDAエラーチェックマクロ
#define CheckCudaErrors(code) { \
  if (code != cudaSuccess) { \
    RCLCPP_ERROR(get_logger(), "CUDA error at %s:%d, Error code: %d (%s)", \
      __FILE__, __LINE__, code, cudaGetErrorString(code)); \
    throw std::runtime_error("CUDA error"); \
  } \
}

MergeSyncNode::MergeSyncNode(const rclcpp::NodeOptions options)
: rclcpp::Node("merge_sync_node", options)
{
  // --- パラメータ ---
  past_len_ = declare_parameter<int>("past_len", 5);
  odom_dim_ = declare_parameter<int>("odom_dim", 8);
  // launchファイル(dnn_image_encoder)の 'final_tensor_name' と一致させる
  image_tensor_name_in_list_ = declare_parameter<std::string>("image_tensor_name_in_list", "input_tensor");
  // ONNXエクスポート時に指定した名前
  image_tensor_name_in_model_ = declare_parameter<std::string>("image_tensor_name_in_model", "input_image");
  odom_tensor_name_in_model_ = declare_parameter<std::string>("odom_tensor_name_in_model", "input_odoms");

  RCLCPP_INFO(get_logger(), "Starting MergeSyncNode...");

  // --- CUDA関連の初期化 ---
  CheckCudaErrors(cudaStreamCreate(&stream_));
  const size_t odom_buffer_size = past_len_ * odom_dim_ * sizeof(float);
  CheckCudaErrors(cudaMallocHost(&odom_cpu_pinned_buffer_, odom_buffer_size));
  CheckCudaErrors(cudaMalloc(&odom_gpu_buffer_, odom_buffer_size));

  // --- QoS ---
  rclcpp::QoS input_qos = ::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "input_qos");
  rclcpp::QoS output_qos = ::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "output_qos");

  // --- パブリッシャ (TritonNodeへ) ---
  nitros_tensor_pub_ = std::make_shared<
    nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosTensorList>>(
    this, "tensor_pub", // 出力トピック (TritonNodeの入力へリマップ)
    nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name,
    output_qos);

  // --- サブスクライバ (オドメトリ) ---
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", input_qos,
    std::bind(&MergeSyncNode::OdomCallback, this, std::placeholders::_1));

  // --- サブスクライバ (画像テンソル) ---
  // NitrosTensorList を購読
  nitros_tensor_sub_ = std::make_shared<
    nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
      nvidia::isaac_ros::nitros::NitrosTensorListView>>(
    this, "tensor", // 入力トピック (dnn_image_encoderの出力へリマップ)
    nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name,
    std::bind(&MergeSyncNode::TensorCallback, this, std::placeholders::_1),
    input_qos);
    
  RCLCPP_INFO(get_logger(), "Node initialized. Subscribing to '%s' (Odom) and '%s' (Tensor).",
    odom_sub_->get_topic_name(), nitros_tensor_sub_->get_topic_name());
}

MergeSyncNode::~MergeSyncNode()
{
  CheckCudaErrors(cudaStreamSynchronize(stream_));
  CheckCudaErrors(cudaFreeHost(odom_cpu_pinned_buffer_));
  CheckCudaErrors(cudaFree(odom_gpu_buffer_));
  CheckCudaErrors(cudaStreamDestroy(stream_));
}

void MergeSyncNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (odom_buffer_.size() >= past_len_) {
    odom_buffer_.pop_front();
  }
  odom_buffer_.push_back({
    static_cast<float>(msg->pose.pose.position.x),
    static_cast<float>(msg->pose.pose.position.y),
    static_cast<float>(msg->pose.pose.position.z),
    static_cast<float>(msg->pose.pose.orientation.x),
    static_cast<float>(msg->pose.pose.orientation.y),
    static_cast<float>(msg->pose.pose.orientation.z),
    static_cast<float>(msg->pose.pose.orientation.w),
    static_cast<float>(msg->twist.twist.linear.x)
  });
}

// ImageCallback -> TensorCallback
void MergeSyncNode::TensorCallback(const nvidia::isaac_ros::nitros::NitrosTensorListView & tensor_list_view)
{
  // --- 1. オドメトリバッファの確認とCPU->GPUコピー ---
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (odom_buffer_.size() < past_len_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 1000, 
        "Odom buffer not full (%ld/%ld), skipping frame.", odom_buffer_.size(), past_len_);
      return;
    }
    // deque -> Pinnedメモリ
    size_t i = 0;
    for (const auto& vec : odom_buffer_) {
      std::copy(vec.begin(), vec.end(), odom_cpu_pinned_buffer_ + (i * odom_dim_));
      i++;
    }
  }
  // Pinnedメモリ -> GPUメモリ
  CheckCudaErrors(cudaMemcpyAsync(
    odom_gpu_buffer_, odom_cpu_pinned_buffer_,
    past_len_ * odom_dim_ * sizeof(float),
    cudaMemcpyHostToDevice, stream_
  ));

  // --- 2. テンソルリストの作成 ---
  std_msgs::msg::Header header = tensor_list_view.GetHeader();

  // 入力TensorListから画像テンソルを取得
  const auto& image_tensor = tensor_list_view.GetTensorView(image_tensor_name_in_list_);
  if (image_tensor.GetData() == nullptr)
  {
    RCLCPP_ERROR(
      get_logger(), "Could not find tensor '%s' in input TensorList.", 
      image_tensor_name_in_list_.c_str());
    return;
  }

  auto tensor_list = nvidia::isaac_ros::nitros::NitrosTensorListBuilder()
    .WithHeader(header)
    
    // --- 3a. 画像テンソルを追加 ---
    // (受け取ったテンソルをそのまま、名前だけ変えて追加)
    .AddTensor(
      image_tensor_name_in_model_, // Triton(ONNX)が期待する名前
      (nvidia::isaac_ros::nitros::NitrosTensorBuilder()
        .WithShape(image_tensor.GetShape()) // 形状をコピー
        .WithDataType(image_tensor.GetDataType()) // 型をコピー
        .WithData(image_tensor.GetData()) // GPUポインタをコピー
        .Build()))
        
    // --- 3b. オドメトリテンソルを追加 ---
    .AddTensor(
      odom_tensor_name_in_model_, // Triton(ONNX)が期待する名前
      (nvidia::isaac_ros::nitros::NitrosTensorBuilder()
        .WithShape({static_cast<int32_t>(past_len_), static_cast<int32_t>(odom_dim_)})
        .WithDataType(nvidia::isaac_ros::nitros::NitrosDataType::kFloat32)
        .WithData(odom_gpu_buffer_) // 自前でH2DコピーしたGPUポインタ
        .Build()))
        
    .Build();

  // --- 4. パブリッシュ ---
  CheckCudaErrors(cudaStreamSynchronize(stream_));
  nitros_tensor_pub_->publish(tensor_list);
}

}  // namespace trajectory_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_preprocessor::MergeSyncNode)