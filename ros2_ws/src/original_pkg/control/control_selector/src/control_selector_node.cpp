#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <memory>

class AckermannCmdSelectorNode : public rclcpp::Node
{
public:
  AckermannCmdSelectorNode() 
    : Node("ackermann_cmd_selector_node"),
      last_log_time_(this->now()),
      log_throttle_duration_(1, 0),  // 1秒 (seconds, nanoseconds)
      message_count_(0)
  {
    // パラメータの宣言
    this->declare_parameter<int>("num_inputs", 3);
    this->declare_parameter<std::string>("input_topic_prefix", "/ackermann_cmd_");
    this->declare_parameter<std::string>("output_topic", "/ackermann_cmd");
    this->declare_parameter<std::string>("section_topic", "current_section");
    
    // パラメータの取得
    int num_inputs = this->get_parameter("num_inputs").as_int();
    std::string input_topic_prefix = this->get_parameter("input_topic_prefix").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string section_topic = this->get_parameter("section_topic").as_string();
    
    // 現在選択されているセクション（デフォルトは0）
    current_section_ = 0;
    
    // 最新のメッセージを保存するバッファを初期化
    latest_msgs_.resize(num_inputs);
    
    // 可変数のsubscriberを作成
    subscribers_.reserve(num_inputs);
    for (int i = 0; i < num_inputs; ++i) {
      std::string topic_name = input_topic_prefix + std::to_string(i + 1);
      
      auto sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        topic_name, 10,
        [this, i](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
          this->ackermannCallback(msg, i);
        });
      
      subscribers_.push_back(sub);
      RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic_name.c_str());
    }
    
    // Publisherの作成
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      output_topic, 10);
    
    // セクション選択用のsubscriberを作成
    section_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      section_topic, 10,
      std::bind(&AckermannCmdSelectorNode::sectionCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Ackermann Command Selector Node initialized");
    RCLCPP_INFO(this->get_logger(), "Number of inputs: %d", num_inputs);
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Section topic: %s", section_topic.c_str());
  }

private:
  void ackermannCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg, 
    int index)
  {
    // 最新のメッセージを保存
    latest_msgs_[index] = msg;
    
    // 現在のセクションに対応するメッセージならpublish
    if (index == current_section_) {
      publisher_->publish(*msg);
      message_count_++;
      
      // ログは1秒に1回だけ出力（スロットル）
      auto current_time = this->now();
      if ((current_time - last_log_time_) >= log_throttle_duration_) {
        RCLCPP_INFO(this->get_logger(), 
          "Section %d: Published %ld messages (speed=%.2f, steering=%.2f)",
          index, message_count_, msg->drive.speed, msg->drive.steering_angle);
        last_log_time_ = current_time;
        message_count_ = 0;
      }
    }
  }
  
  void sectionCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int new_section = msg->data;
    
    // セクション番号の範囲チェック
    if (new_section < 0 || new_section >= static_cast<int>(subscribers_.size())) {
      RCLCPP_WARN(this->get_logger(), 
        "Invalid section number: %d (valid range: 0-%lu)",
        new_section, subscribers_.size() - 1);
      return;
    }
    
    if (new_section != current_section_) {
      RCLCPP_INFO(this->get_logger(), 
        "Section changed: %d -> %d", current_section_, new_section);
      current_section_ = new_section;
      
      // セクション切り替え時に最新のメッセージがあればpublish
      if (latest_msgs_[current_section_]) {
        publisher_->publish(*latest_msgs_[current_section_]);
        RCLCPP_INFO(this->get_logger(), 
          "Published latest message from new section %d", current_section_);
      }
    }
  }
  
  // メンバ変数
  std::vector<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr> subscribers_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr section_sub_;
  
  std::vector<ackermann_msgs::msg::AckermannDriveStamped::SharedPtr> latest_msgs_;
  int current_section_;
  
  // ログ頻度制御用
  rclcpp::Time last_log_time_;
  rclcpp::Duration log_throttle_duration_;
  size_t message_count_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AckermannCmdSelectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}