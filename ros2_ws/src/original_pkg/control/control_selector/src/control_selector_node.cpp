#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

#include <string>
#include <vector>
#include <memory>

/**
 * @brief "mode" (int) に応じて、複数の Ackermann 指令トピックから
 * 1つを選択してパブリッシュするノード。
 */
class AckermannModeSelectorNode : public rclcpp::Node
{
public:
  AckermannModeSelectorNode()
  : Node("ackermann_mode_selector_node"),
    current_mode_(-1) // 未初期化
  {
    // ============================
    //  パラメータ宣言
    // ============================
    this->declare_parameter<std::vector<std::string>>(
      "input_topics",
      std::vector<std::string>({}));
    this->declare_parameter<std::string>("output_topic", "/ackermann_cmd");
    this->declare_parameter<std::string>("mode_topic", "/current_mode");
    this->declare_parameter<int>("default_mode", 0);


    // ============================
    //  パラメータ取得
    // ============================
    std::vector<std::string> input_topics =
      this->get_parameter("input_topics").as_string_array();
    const std::string output_topic =
      this->get_parameter("output_topic").as_string();
    const std::string mode_topic =
      this->get_parameter("mode_topic").as_string();
    
    current_mode_ = this->get_parameter("default_mode").as_int();


    if (input_topics.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No 'input_topics' defined! Use e.g. ['/ackermann_cmd_0', '/ackermann_cmd_1']");
      rclcpp::shutdown();
      return;
    }

    // ============================
    //  動的に Subscriber を作成
    // ============================
    latest_msgs_.resize(input_topics.size(), nullptr);

    for (size_t i = 0; i < input_topics.size(); ++i) {
      const std::string &topic_name = input_topics[i];
      const int mode_index = static_cast<int>(i);

      RCLCPP_INFO(this->get_logger(), "Registering Mode %d -> %s",
                  mode_index, topic_name.c_str());

      auto sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        topic_name, rclcpp::SystemDefaultsQoS(),
        [this, mode_index](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
          this->ackermannCallback(msg, mode_index);
        });

      subscribers_.push_back(sub);
    }

    // ============================
    //  出力・モード購読
    // ============================
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      output_topic, rclcpp::SystemDefaultsQoS());

    mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      mode_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AckermannModeSelectorNode::modeCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "AckermannModeSelectorNode initialized with %zu inputs.",
                input_topics.size());
  }

private:
  /**
   * @brief Ackermann 指令が入力されたときのコールバック
   * @param mode_index このトピックが担当するモード番号
   */
  void ackermannCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg,
    int mode_index)
  {
    // 最新のメッセージを保存
    if (mode_index >= 0 && static_cast<size_t>(mode_index) < latest_msgs_.size()) {
      latest_msgs_[mode_index] = msg;
    }

    // このトピックが現在の選択モードと一致すれば、そのままパブリッシュ
    if (mode_index == current_mode_) {
      publisher_->publish(*msg);
    }
  }

  /**
   * @brief モード切り替え指示のコールバック
   */
  void modeCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const int new_mode = msg->data;

    // モードが範囲外
    if (new_mode < 0 || static_cast<size_t>(new_mode) >= latest_msgs_.size()) {
      RCLCPP_WARN(this->get_logger(), "Invalid mode %d requested (max is %zu)",
                  new_mode, latest_msgs_.size() - 1);
      return;
    }

    // モードが変更された
    if (new_mode != current_mode_) {
      RCLCPP_INFO(this->get_logger(), "Switching to Mode %d", new_mode);
      current_mode_ = new_mode;

      // モード切り替え時、そのモードで最後に受信したメッセージを即座に再送する
      auto last_msg = latest_msgs_[current_mode_];
      if (last_msg) {
        publisher_->publish(*last_msg);
        RCLCPP_INFO(this->get_logger(), "Published last known msg for mode %d", current_mode_);
      } else {
        RCLCPP_WARN(this->get_logger(), "No latest message yet for mode %d", current_mode_);
      }
    }
  }

  // メンバ変数
  std::vector<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr> subscribers_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

  std::vector<ackermann_msgs::msg::AckermannDriveStamped::SharedPtr> latest_msgs_;
  int current_mode_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannModeSelectorNode>());
  rclcpp::shutdown();
  return 0;
}