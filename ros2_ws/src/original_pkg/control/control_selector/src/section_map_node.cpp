#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cctype> // for std::isspace

/**
 * @brief "key:value" 形式の文字列をパースし、
 * (int) section -> (int) mode の対応関係を管理するノード。
 */
class SectionMapperNode : public rclcpp::Node
{
public:
  SectionMapperNode()
  : Node("section_mapper_node"),
    current_mode_(-1) // 未初期化を示す値
  {
    // ============================
    //  パラメータ宣言
    // ============================
    this->declare_parameter<std::vector<std::string>>(
      "section_to_mode_map",
      std::vector<std::string>({}));
    this->declare_parameter<std::string>("section_topic", "/current_section");
    this->declare_parameter<std::string>("mode_topic", "/current_mode");

    // ============================
    //  パラメータ取得
    // ============================
    std::vector<std::string> section_to_mode_vec =
      this->get_parameter("section_to_mode_map").as_string_array();
    const std::string section_topic =
      this->get_parameter("section_topic").as_string();
    const std::string mode_topic =
      this->get_parameter("mode_topic").as_string();

    // ============================
    //  マッピングのパース ("key:value" -> map<int, int>)
    // ============================
    auto trim = [](std::string s) {
      auto is_ws = [](unsigned char c){ return std::isspace(c); };
      while (!s.empty() && is_ws(s.front())) s.erase(s.begin());
      while (!s.empty() && is_ws(s.back()))  s.pop_back();
      return s;
    };

    auto split_kv_int = [&](const std::string &line, char delim)
      -> std::pair<int, int>
    {
      const auto pos = line.find(delim);
      if (pos == std::string::npos) return {-1, -1};
      std::string k_str = trim(line.substr(0, pos));
      std::string v_str = trim(line.substr(pos + 1));

      try {
        return {std::stoi(k_str), std::stoi(v_str)};
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
          "Invalid map item '%s': %s", line.c_str(), e.what());
        return {-1, -1};
      }
    };

    for (const auto &kv_line : section_to_mode_vec) {
      auto kv = split_kv_int(kv_line, ':');
      if (kv.first >= 0 && kv.second >= 0) {
        section_to_mode_[kv.first] = kv.second;
        RCLCPP_INFO(this->get_logger(),
          "Mapping Section %d -> Mode %d", kv.first, kv.second);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Ignoring invalid 'section_to_mode_map' item: '%s'", kv_line.c_str());
      }
    }

    if (section_to_mode_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No valid 'section_to_mode_map' defined!");
      rclcpp::shutdown();
      return;
    }

    // ============================
    //  Publisher / Subscriber
    // ============================
    mode_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      mode_topic, rclcpp::SystemDefaultsQoS());

    section_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      section_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&SectionMapperNode::sectionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SectionMapperNode initialized.");
  }

private:
  void sectionCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const int section = msg->data;

    auto it = section_to_mode_.find(section);
    if (it == section_to_mode_.end()) {
      // マッピングにないセクションが来た場合は、警告を出して何もしない
      // (現在のモードを継続する)
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Unknown section %d (no mode mapping)", section);
      return;
    }

    const int new_mode = it->second;

    // モードが変更された場合のみパブリッシュ
    if (new_mode != current_mode_) {
      RCLCPP_INFO(this->get_logger(),
        "Section %d -> Switching to Mode %d", section, new_mode);
      current_mode_ = new_mode;

      auto mode_msg = std_msgs::msg::Int32();
      mode_msg.data = current_mode_;
      mode_pub_->publish(mode_msg);
    }
  }

  // メンバ変数
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr section_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;

  std::map<int, int> section_to_mode_;
  int current_mode_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SectionMapperNode>());
  rclcpp::shutdown();
  return 0;
}