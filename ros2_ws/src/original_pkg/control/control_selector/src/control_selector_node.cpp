#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

#include <unordered_map>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cctype>

class AckermannCmdSelectorNode : public rclcpp::Node
{
public:
  AckermannCmdSelectorNode()
  : Node("ackermann_cmd_selector_node"),
    last_log_time_(this->now()),
    log_throttle_duration_(1, 0),
    message_count_(0)
  {
    // ============================
    //  パラメータ宣言（vector<string>で受ける）
    // ============================
    this->declare_parameter<std::vector<std::string>>("input_topics", {});
    this->declare_parameter<std::vector<std::string>>("section_to_mode", {});
    this->declare_parameter<std::string>("output_topic", "/ackermann_cmd");
    this->declare_parameter<std::string>("section_topic", "current_section");

    // ============================
    //  パラメータ取得
    // ============================
    std::vector<std::string> input_topics_vec =
      this->get_parameter("input_topics").as_string_array();
    std::vector<std::string> section_to_mode_vec =
      this->get_parameter("section_to_mode").as_string_array();
    const std::string output_topic =
      this->get_parameter("output_topic").as_string();
    const std::string section_topic =
      this->get_parameter("section_topic").as_string();

    // ============================
    //  文字列 "key:value" の配列 → map に変換
    // ============================
    auto trim = [](std::string s) {
      auto is_ws = [](unsigned char c){ return std::isspace(c); };
      while (!s.empty() && is_ws(s.front())) s.erase(s.begin());
      while (!s.empty() && is_ws(s.back()))  s.pop_back();
      return s;
    };

    auto split_kv = [&](const std::string &line, char delim)
      -> std::pair<std::string, std::string>
    {
      const auto pos = line.find(delim);
      if (pos == std::string::npos) return {"", ""};
      std::string k = trim(line.substr(0, pos));
      std::string v = trim(line.substr(pos + 1));
      return {k, v};
    };

    std::map<std::string, std::string> input_topics_param;
    for (const auto &item : input_topics_vec) {
      auto kv = split_kv(item, ':');
      if (kv.first.empty() || kv.second.empty()) {
        RCLCPP_WARN(this->get_logger(),
          "Invalid 'input_topics' item: '%s' (expected 'mode:topic')", item.c_str());
        continue;
      }
      input_topics_param[kv.first] = kv.second;
    }

    if (input_topics_param.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No valid 'input_topics' defined! Use e.g. ['slow_drive:/ackermann_cmd_1', ...]");
      rclcpp::shutdown();
      return;
    }

    for (const auto &kvline : section_to_mode_vec) {
      auto kv = split_kv(kvline, ':');
      if (kv.first.empty() || kv.second.empty()) {
        RCLCPP_WARN(this->get_logger(),
          "Invalid 'section_to_mode' item: '%s' (expected 'section:int_mode')", kvline.c_str());
        continue;
      }
      try {
        int section = std::stoi(kv.first);
        section_to_mode_[section] = kv.second;
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
          "Invalid section index in 'section_to_mode': '%s' (%s)", kv.first.c_str(), e.what());
      }
    }

    // ============================
    //  動的に Subscriber を作成
    // ============================
    int index = 0;
    for (const auto &pair : input_topics_param) {
      const std::string &mode_name = pair.first;
      const std::string &topic_name = pair.second;

      mode_to_index_[mode_name] = index;
      RCLCPP_INFO(this->get_logger(), "Registering mode '%s' -> %s",
                  mode_name.c_str(), topic_name.c_str());

      auto sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        topic_name, rclcpp::SystemDefaultsQoS(),
        [this, index, mode_name](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
          this->ackermannCallback(msg, index, mode_name);
        });

      subscribers_.push_back(sub);
      latest_msgs_.push_back(nullptr);
      ++index;
    }

    // ============================
    //  出力・セクション購読
    // ============================
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      output_topic, rclcpp::SystemDefaultsQoS());

    section_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      section_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AckermannCmdSelectorNode::sectionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "AckermannCmdSelectorNode initialized.");
    RCLCPP_INFO(this->get_logger(), "Loaded %zu input topics, %zu section mappings.",
                input_topics_param.size(), section_to_mode_.size());
  }

private:
  void ackermannCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg,
    int index,
    const std::string &mode)
  {
    if (index >= 0 && static_cast<size_t>(index) < latest_msgs_.size()) {
      latest_msgs_[index] = msg;
    }

    if (mode == current_mode_) {
      publisher_->publish(*msg);
      message_count_++;

      auto now = this->now();
      if ((now - last_log_time_) >= log_throttle_duration_) {
        RCLCPP_INFO(this->get_logger(),
          "[%s] Published %zu msgs (speed=%.2f, steering=%.2f)",
          mode.c_str(), message_count_,
          msg->drive.speed, msg->drive.steering_angle);
        last_log_time_ = now;
        message_count_ = 0;
      }
    }
  }

  void sectionCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const int section = msg->data;

    auto it = section_to_mode_.find(section);
    if (it == section_to_mode_.end()) {
      RCLCPP_WARN(this->get_logger(), "Unknown section %d (no mode mapping)", section);
      return;
    }

    const std::string new_mode = it->second;
    if (new_mode != current_mode_) {
      RCLCPP_INFO(this->get_logger(), "Section %d -> Mode '%s'", section, new_mode.c_str());
      current_mode_ = new_mode;

      auto mit = mode_to_index_.find(new_mode);
      if (mit != mode_to_index_.end()) {
        const int idx = mit->second;
        if (idx >= 0 && static_cast<size_t>(idx) < latest_msgs_.size() && latest_msgs_[idx]) {
          publisher_->publish(*latest_msgs_[idx]);
          RCLCPP_INFO(this->get_logger(), "Published last known msg for mode '%s'", new_mode.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "No latest message yet for mode '%s'", new_mode.c_str());
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Mode '%s' not found in input_topics list", new_mode.c_str());
      }
    }
  }

  // メンバ変数
  std::vector<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr> subscribers_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr section_sub_;

  std::unordered_map<std::string, int> mode_to_index_;
  std::map<int, std::string> section_to_mode_;
  std::vector<ackermann_msgs::msg::AckermannDriveStamped::SharedPtr> latest_msgs_;
  std::string current_mode_;

  rclcpp::Time last_log_time_;
  rclcpp::Duration log_throttle_duration_;
  size_t message_count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannCmdSelectorNode>());
  rclcpp::shutdown();
  return 0;
}
