#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/int32.hpp> // セクション購読のため追加
#include <deque>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <functional>
#include <cmath>
#include <map> // パラメータ保持のため追加
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// トピック名のデフォルト値
const char* DRIVE_INPUT_TOPIC = "/cmd_drive";
const char* DRIVE_OUTPUT_TOPIC = "/cmd_drive_filtered";
const char* SECTION_INPUT_TOPIC = "/current_section";

/**
 * @brief セクションごとの制御パラメータを保持する構造体
 * (元のAckermannFilterNodeのパラメータ + steer_offset)
 */
struct SectionParams
{
  // スケーリング共通
  bool use_scale_filter = true;
  std::string scale_filter_type = "normal";
  double steer_offset = 0.0; 

  // "normal" モード用
  double normal_speed_scale_ratio = 1.0;
  double normal_steer_scale_ratio = 1.0;

  // "advance" モード用
  double advance_straight_steer_threshold = 0.1;
  double advance_straight_speed_scale_ratio = 1.0;
  double advance_cornering_speed_scale_ratio = 0.5;
  double advance_steer_scale_ratio = 1.0;
};

/**
 * @brief セクションに応じてパラメータを切り替えるAckermannフィルタノード
 */
class SectionalAckermannFilterNode : public rclcpp::Node
{
public:
  SectionalAckermannFilterNode()
  : Node("sectional_ackermann_filter_node"), // ノード名を変更
    current_section_(-1) // 未初期化
  {
    // ============================
    //  パラメータ宣言 (平滑化フィルタ)
    // ============================
    this->declare_parameter<std::string>("filter_type", "none");
    this->declare_parameter<int>("window_size", 5);

    // ============================
    //  パラメータ宣言 (トピック名)
    // ============================
    this->declare_parameter<std::string>("drive_input_topic", DRIVE_INPUT_TOPIC);
    this->declare_parameter<std::string>("drive_output_topic", DRIVE_OUTPUT_TOPIC);
    this->declare_parameter<std::string>("section_input_topic", SECTION_INPUT_TOPIC);

    // ============================
    //  パラメータ宣言 (セクション定義)
    // ============================
    this->declare_parameter<std::vector<int64_t>>(
      "defined_sections",
      std::vector<int64_t>({0})); // デフォルト: Section 0 のみ

    // パラメータ取得 (平滑化)
    this->get_parameter("filter_type", filter_type_);
    this->get_parameter("window_size", window_size_);

    // パラメータ取得 (トピック名) -> メンバ変数に格納
    drive_input_topic_ = this->get_parameter("drive_input_topic").as_string();
    drive_output_topic_ = this->get_parameter("drive_output_topic").as_string();
    section_input_topic_ = this->get_parameter("section_input_topic").as_string();
    
    // 起動時デバッグプリント (トピック名)
    RCLCPP_INFO(this->get_logger(), "--- Topic Settings ---");
    RCLCPP_INFO(this->get_logger(), "Drive Input:   '%s'", drive_input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Drive Output:  '%s'", drive_output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Section Input: '%s'", section_input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "----------------------");

    // ============================
    //  セクションごとのパラメータを宣言・取得
    // ============================
    std::vector<int64_t> defined_sections =
      this->get_parameter("defined_sections").as_integer_array();

    if (defined_sections.empty()) {
       RCLCPP_WARN(this->get_logger(), 
         "Parameter 'defined_sections' is empty. Loading for section 0 only.");
       defined_sections.push_back(0);
    }

    RCLCPP_INFO(this->get_logger(), "--- Loading Section Parameters ---"); 
    for (int64_t section_id_long : defined_sections) {
      int section_id = static_cast<int>(section_id_long);
      std::string prefix = "sections." + std::to_string(section_id) + ".";
      
      SectionParams params;
      // 構造体にパラメータを宣言・格納
      params.use_scale_filter = this->declare_parameter<bool>(prefix + "use_scale_filter", true);
      params.scale_filter_type = this->declare_parameter<std::string>(prefix + "scale_filter_type", "normal");
      params.steer_offset = this->declare_parameter<double>(prefix + "steer_offset", 0.0); 

      params.normal_speed_scale_ratio = this->declare_parameter<double>(prefix + "normal.speed_scale_ratio", 1.0);
      params.normal_steer_scale_ratio = this->declare_parameter<double>(prefix + "normal.steer_scale_ratio", 1.0);

      params.advance_straight_steer_threshold = this->declare_parameter<double>(prefix + "advance.straight_steer_threshold", 0.1);
      params.advance_straight_speed_scale_ratio = this->declare_parameter<double>(prefix + "advance.straight_speed_scale_ratio", 1.0);
      params.advance_cornering_speed_scale_ratio = this->declare_parameter<double>(prefix + "advance.cornering_speed_scale_ratio", 0.5);
      params.advance_steer_scale_ratio = this->declare_parameter<double>(prefix + "advance.steer_scale_ratio", 1.0);
      
      section_params_[section_id] = params; // セクションIDをキーとしてマップに保存
      RCLCPP_INFO(this->get_logger(), "Loaded params for Section %d (Filter: %s, Offset: %.2f)", 
        section_id, params.scale_filter_type.c_str(), params.steer_offset);
    }
    RCLCPP_INFO(this->get_logger(), "------------------------------------"); 

    // ============================
    //  Publisher / Subscriber / Callback
    // ============================
    parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SectionalAckermannFilterNode::parameters_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_output_topic_, 10); 
    
    drive_subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_input_topic_, 10, std::bind(&SectionalAckermannFilterNode::drive_callback, this, std::placeholders::_1));

    section_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      section_input_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&SectionalAckermannFilterNode::section_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SectionalAckermannFilterNode initialized.");
    RCLCPP_INFO(this->get_logger(), "Waiting for first section message on '%s' to activate parameters...", section_input_topic_.c_str());
    
    // ★ 削除: 起動時の冗長な設定表示を削除
    // print_current_settings(); 
  }

private:
  /**
   * @brief 制御指令（/cmd_drive）受信時の処理
   */
  void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    // 1. バッファリング
    speed_buffer_.push_back(msg->drive.speed);
    steering_angle_buffer_.push_back(msg->drive.steering_angle);

    while (speed_buffer_.size() > static_cast<size_t>(window_size_)) {
      speed_buffer_.pop_front();
      steering_angle_buffer_.pop_front();
    }

    auto filtered_drive_data = ackermann_msgs::msg::AckermannDrive();
    
    // 2. 平滑化フィルタ (Average / Median / None)
    // (filter_type_ は "none", "average", "median" のいずれかを想定)
    if (filter_type_ == "average") { 
      apply_average_filter(filtered_drive_data);
    } else if (filter_type_ == "median") {
      apply_median_filter(filtered_drive_data);
    } else { // "none" または未定義の文字列の場合
      if (!speed_buffer_.empty()) {
        filtered_drive_data.speed = speed_buffer_.back();
        filtered_drive_data.steering_angle = steering_angle_buffer_.back();
      } else {
        filtered_drive_data.speed = msg->drive.speed;
        filtered_drive_data.steering_angle = msg->drive.steering_angle;
      }
    }
    
    // 3. セクションに応じたスケーリングとオフセットを適用
    // 現在のセクションに対応するパラメータセットを取得
    auto it = section_params_.find(current_section_);
    if (it == section_params_.end()) {
      // パラメータが見つからない場合 (未定義セクションなど)
      // クランプ処理のみ行う
      clamp_drive_data(filtered_drive_data);
    } else {
      const SectionParams &params = it->second;
      
      // スケーリングフィルタを適用
      if (params.use_scale_filter) {
        if (params.scale_filter_type == "advance") {
            apply_advanced_scale_filter(filtered_drive_data, params);
        } else { // "normal"
            apply_normal_scale_filter(filtered_drive_data, params);
        }
      }

      // オフセットを加算
      filtered_drive_data.steering_angle += params.steer_offset;

      // 最終的なクランプ処理
      clamp_drive_data(filtered_drive_data);
    }
    
    // 4. 配信
    auto filtered_stamped_msg = ackermann_msgs::msg::AckermannDriveStamped();
    filtered_stamped_msg.header = msg->header;
    filtered_stamped_msg.drive = filtered_drive_data;
    publisher_->publish(filtered_stamped_msg);
  }
  
  /**
   * @brief セクション番号 (/current_section) 受信時の処理 
   */
  void section_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const int new_section = msg->data;

    if (new_section != current_section_) {
      if (section_params_.find(new_section) == section_params_.end()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Received Section %d, but no parameters (sections.%d.*) are defined. "
          "Using parameters from previous section %d.",
          new_section, new_section, current_section_);
        // パラメータが定義されていないセクションに来たら、前のセクションの設定を維持
        return; 
      }

      // ★修正: 以前のセクションが-1（未初期化）でなければ、モード変更ログを出す
      if (current_section_ != -1) {
          RCLCPP_INFO(this->get_logger(),
            "Section %d -> Activating parameters.", new_section);
      } else {
          RCLCPP_INFO(this->get_logger(),
            "Received first section %d -> Activating parameters.", new_section);
      }
      current_section_ = new_section;
      print_current_settings(); // ★設定がアクティブになった/変更されたここで初めて表示
    }
  }

  /**
   * @brief パラメータ動的変更時の処理 
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    bool settings_changed = false;

    for (const auto &param : parameters) {
      const std::string param_name = param.get_name();
      
      // ★ 削除: 冗長なログをコメントアウト
      // RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed.", param_name.c_str());

      if (param_name == "filter_type") {
        filter_type_ = param.as_string();
        settings_changed = true;
      } else if (param_name == "window_size") {
        window_size_ = param.as_int();
        settings_changed = true;
      } 
      // "sections.S.xxx" 形式のパラメータ変更を処理
      else if (param_name.rfind("sections.", 0) == 0) {
        settings_changed = true;
        // 例: "sections.1.normal.speed_scale_ratio"
        std::string name = param_name.substr(9); // "1.normal.speed_scale_ratio"
        size_t dot_pos = name.find('.');
        if (dot_pos == std::string::npos) continue;
        
        try {
          int section_id = std::stoi(name.substr(0, dot_pos)); // "1" -> 1
          if (!section_params_.count(section_id)) continue; // 未定義セクションは無視

          std::string key = name.substr(dot_pos + 1); // "normal.speed_scale_ratio"
          
          // ★修正: どのパラメータが変更されたかを表示 (これは有用)
          RCLCPP_INFO(this->get_logger(), "Updating param for Section %d: %s = %s", 
            section_id, key.c_str(), param.value_to_string().c_str()); 

          // 対応するセクションのパラメータマップを更新
          if (key == "use_scale_filter") {
            section_params_[section_id].use_scale_filter = param.as_bool();
          } else if (key == "scale_filter_type") {
            section_params_[section_id].scale_filter_type = param.as_string();
          } else if (key == "steer_offset") {
            section_params_[section_id].steer_offset = param.as_double();
          } else if (key == "normal.speed_scale_ratio") {
            section_params_[section_id].normal_speed_scale_ratio = param.as_double();
          } else if (key == "normal.steer_scale_ratio") {
            section_params_[section_id].normal_steer_scale_ratio = param.as_double();
          } else if (key == "advance.straight_steer_threshold") {
            section_params_[section_id].advance_straight_steer_threshold = param.as_double();
          } else if (key == "advance.straight_speed_scale_ratio") {
            section_params_[section_id].advance_straight_speed_scale_ratio = param.as_double();
          } else if (key == "advance.cornering_speed_scale_ratio") {
            section_params_[section_id].advance_cornering_speed_scale_ratio = param.as_double();
          } else if (key == "advance.steer_scale_ratio") {
            section_params_[section_id].advance_steer_scale_ratio = param.as_double();
          }
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse param '%s': %s", 
              param_name.c_str(), e.what());
        }
      }
    }
    
    if (result.successful && settings_changed) {
        RCLCPP_INFO(this->get_logger(), "New parameters have been applied.");
        // ★修正: 変更が適用されたセクションが現在アクティブな場合のみ、設定を再表示する
        if (current_section_ != -1) {
             print_current_settings();
        }
    }

    return result;
  }
  
  /**
   * @brief 現在の設定をコンソールに出力 (トピック名表示を追加)
   */
  void print_current_settings() {
    RCLCPP_INFO(this->get_logger(), "--- Sectional Ackermann Filter Settings ---");
    // トピック名も表示
    RCLCPP_INFO(this->get_logger(), "Topics: [In: %s] [Out: %s] [Section: %s]",
        drive_input_topic_.c_str(), drive_output_topic_.c_str(), section_input_topic_.c_str());
        
    RCLCPP_INFO(this->get_logger(), "Global Filter type: %s", filter_type_.c_str());
    if (filter_type_ != "none") {
      RCLCPP_INFO(this->get_logger(), "Global Window size: %d", window_size_);
    }
    RCLCPP_INFO(this->get_logger(), "Current Active Section: %d", current_section_);
    
    // 現在のセクションのパラメータを表示
    if (section_params_.count(current_section_)) {
        const auto& params = section_params_.at(current_section_);
        RCLCPP_INFO(this->get_logger(), "  [Section %d] Use scale filter: %s", 
                    current_section_, params.use_scale_filter ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  [Section %d] Scale filter type: %s", 
                    current_section_, params.scale_filter_type.c_str());
        RCLCPP_INFO(this->get_logger(), "  [Section %d] Steer Offset: %.2f", 
                    current_section_, params.steer_offset);

        if (params.scale_filter_type == "advance") {
          RCLCPP_INFO(this->get_logger(), "    [advance] Threshold: %.2f rad", params.advance_straight_steer_threshold);
          RCLCPP_INFO(this->get_logger(), "    [advance] Straight Speed Scale: %.2f", params.advance_straight_speed_scale_ratio);
          RCLCPP_INFO(this->get_logger(), "    [advance] Cornering Speed Scale: %.2f", params.advance_cornering_speed_scale_ratio);
          RCLCPP_INFO(this->get_logger(), "    [advance] Steer Scale: %.2f", params.advance_steer_scale_ratio);
        } else { // normal
          RCLCPP_INFO(this->get_logger(), "    [normal] Speed Scale: %.2f", params.normal_speed_scale_ratio);
          RCLCPP_INFO(this->get_logger(), "    [normal] Steer Scale: %.2f", params.normal_steer_scale_ratio);
        }
    } else if (current_section_ != -1) {
        RCLCPP_WARN(this->get_logger(), "  [Section %d] Parameters not found! Using previous settings or defaults.", current_section_);
    } else {
        RCLCPP_INFO(this->get_logger(), "  (No section active yet. Waiting for message...)"); 
    }
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------");
  }

  // ========================================
  // フィルタリング関数 
  // ========================================

  void apply_normal_scale_filter(
    ackermann_msgs::msg::AckermannDrive &msg, const SectionParams &params)
  {
    msg.speed *= params.normal_speed_scale_ratio;
    msg.steering_angle *= params.normal_steer_scale_ratio;
    // クランプとオフセット適用は呼び出し元で行う
  }

  void apply_advanced_scale_filter(
    ackermann_msgs::msg::AckermannDrive &msg, const SectionParams &params)
  {
    if (std::fabs(msg.steering_angle) < params.advance_straight_steer_threshold) {
        msg.speed *= params.advance_straight_speed_scale_ratio;
    } else {
        msg.speed *= params.advance_cornering_speed_scale_ratio;
    }
    msg.steering_angle *= params.advance_steer_scale_ratio;
    // クランプとオフセット適用は呼び出し元で行う
  }

  void clamp_drive_data(ackermann_msgs::msg::AckermannDrive &msg)
  {
    msg.speed = std::max(-1.0f, std::min(msg.speed, 1.0f));
    msg.steering_angle = std::max(-1.0f, std::min(msg.steering_angle, 1.0f));
  }

  // --- 平滑化関数 (変更なし) ---
  void apply_average_filter(ackermann_msgs::msg::AckermannDrive &msg)
  {
    if (speed_buffer_.empty()) return;
    double speed_sum = std::accumulate(speed_buffer_.begin(), speed_buffer_.end(), 0.0);
    msg.speed = speed_sum / speed_buffer_.size();
    double steer_sum = std::accumulate(steering_angle_buffer_.begin(), steering_angle_buffer_.end(), 0.0);
    msg.steering_angle = steer_sum / steering_angle_buffer_.size();
  }
  void apply_median_filter(ackermann_msgs::msg::AckermannDrive &msg)
  {
    if (speed_buffer_.empty()) return;
    msg.speed = calculate_median(speed_buffer_);
    msg.steering_angle = calculate_median(steering_angle_buffer_);
  }
  double calculate_median(const std::deque<double>& data)
  {
    if (data.empty()) return 0.0;
    std::vector<double> sorted_data(data.begin(), data.end());
    size_t n = sorted_data.size();
    std::sort(sorted_data.begin(), sorted_data.end());
    if (n % 2 == 0) {
        return (sorted_data[n / 2 - 1] + sorted_data[n / 2]) / 2.0;
    } else {
        return sorted_data[n / 2];
    }
  }

  // ========================================
  // メンバ変数
  // ========================================

  // Pub/Sub
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr section_subscription_; 
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  
  // 平滑化フィルタ用
  std::string filter_type_;
  int window_size_;
  std::deque<double> speed_buffer_;
  std::deque<double> steering_angle_buffer_;

  // セクション管理用
  std::map<int, SectionParams> section_params_; // <Section ID, Parameters>
  int current_section_;

  // デバッグプリント用にトピック名をメンバ変数として保持
  std::string drive_input_topic_;
  std::string drive_output_topic_;
  std::string section_input_topic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SectionalAckermannFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}