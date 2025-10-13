#include <algorithm>
#include <chrono>
#include <memory>
#include <string> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyManagerNode : public rclcpp::Node
{
public:
  JoyManagerNode()
  : Node("joy_manager_node"),
    joy_active_(false),
    ack_active_(false),
    joy_speed_(0.0),
    joy_steer_(0.0),
    prev_start_pressed_(false),
    prev_stop_pressed_(false),
    prev_steer_inc_pressed_(false),
    prev_steer_dec_pressed_(false),
    prev_speed_inc_pressed_(false),
    prev_speed_dec_pressed_(false),
    prev_scale_inc_pressed_(false),
    prev_scale_dec_pressed_(false)
  {
    // --- パラメータ宣言＆取得 ---
    declare_parameter<double>("speed_scale", 1.0);
    declare_parameter<double>("steer_scale", 1.0);
    declare_parameter<int>("joy_button_index",   2);
    declare_parameter<int>("ack_button_index",   3);
    declare_parameter<int>("start_button_index", 9);
    declare_parameter<int>("stop_button_index",  8);
    declare_parameter<int>("speed_axis_index", 1);    
    declare_parameter<int>("steer_axis_index", 3);
    declare_parameter<double>("timer_hz", 40.0);
    declare_parameter<double>("joy_timeout_sec", 0.5);
    declare_parameter<std::string>("frame_id", "base_link");


    get_parameter("speed_scale", speed_scale_);
    get_parameter("steer_scale", steer_scale_);
    get_parameter("joy_button_index", joy_button_index_);
    get_parameter("ack_button_index", ack_button_index_);
    get_parameter("start_button_index", start_button_index_);
    get_parameter("stop_button_index", stop_button_index_);
    get_parameter("speed_axis_index", speed_axis_index_); 
    get_parameter("steer_axis_index", steer_axis_index_);
    get_parameter("timer_hz", timer_hz_);
    get_parameter("joy_timeout_sec", joy_timeout_sec_);
    get_parameter("frame_id", frame_id_); 

    last_autonomy_msg_.speed = 0.0;
    last_autonomy_msg_.steering_angle = 0.0;
    last_joy_msg_time_ = this->get_clock()->now();

    // --- サブスクライバ／パブリッシャ設定 ---
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyManagerNode::joy_callback, this, _1));
    ack_sub_ = create_subscription<ackermann_msgs::msg::AckermannDrive>(
      "/ackermann_cmd", 10, std::bind(&JoyManagerNode::ack_callback, this, _1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd_drive", 10);
    trigger_pub_ = create_publisher<std_msgs::msg::Bool>("/rosbag2_recorder/trigger", 10);

    // オフセット調整用トリガー
    steer_inc_pub_ = create_publisher<std_msgs::msg::Bool>("/steer_offset_inc", 10);
    steer_dec_pub_ = create_publisher<std_msgs::msg::Bool>("/steer_offset_dec", 10);
    speed_inc_pub_ = create_publisher<std_msgs::msg::Bool>("/speed_offset_inc", 10);
    speed_dec_pub_ = create_publisher<std_msgs::msg::Bool>("/speed_offset_dec", 10);

    // --- タイマー（一定周期でコマンド出力）---
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / timer_hz_),
      std::bind(&JoyManagerNode::timer_callback, this));
  }

private:
  // デバウンス付きボタン検出ヘルパー
  bool check_button_press(bool curr, bool &prev_flag) {
    if (curr && !prev_flag) {
      prev_flag = true;
      return true;
    } else if (!curr) {
      prev_flag = false;
    }
    return false;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_msg_time_ = this->get_clock()->now(); 

    bool curr_start = (msg->buttons.size() > start_button_index_
                       && msg->buttons[start_button_index_] == 1);
    bool curr_stop  = (msg->buttons.size() > stop_button_index_
                       && msg->buttons[stop_button_index_]  == 1);
    if (check_button_press(curr_start, prev_start_pressed_)) {
      std_msgs::msg::Bool b; b.data = true;
      trigger_pub_->publish(b);
    }
    if (check_button_press(curr_stop, prev_stop_pressed_)) {
      std_msgs::msg::Bool b; b.data = false;
      trigger_pub_->publish(b);
    }

    bool joy_pressed = (msg->buttons.size() > joy_button_index_
                        && msg->buttons[joy_button_index_] == 1);
    bool ack_pressed = (msg->buttons.size() > ack_button_index_
                        && msg->buttons[ack_button_index_] == 1);
    if (ack_pressed) {
      ack_active_ = true; joy_active_ = false;
    } else if (joy_pressed) {
      joy_active_ = true; ack_active_ = false;
    } else {
      joy_active_ = false; ack_active_ = false;
    }

    if (joy_active_) {
      double raw_speed = (static_cast<int>(msg->axes.size()) > speed_axis_index_ ? msg->axes[speed_axis_index_] : 0.0);
      double raw_steer = (static_cast<int>(msg->axes.size()) > steer_axis_index_ ? msg->axes[steer_axis_index_] : 0.0);
      joy_speed_ = raw_speed * speed_scale_;
      joy_steer_ = raw_steer * steer_scale_;
    }

    double a6 = (msg->axes.size() > 6 ? msg->axes[6] : 0.0);
    double a7 = (msg->axes.size() > 7 ? msg->axes[7] : 0.0);

    bool steer_inc = std::abs(a6 + 1.0) < 1e-3;
    bool steer_dec = std::abs(a6 - 1.0) < 1e-3;
    bool speed_inc = std::abs(a7 - 1.0) < 1e-3;
    bool speed_dec = std::abs(a7 + 1.0) < 1e-3;

    if (check_button_press(steer_inc, prev_steer_inc_pressed_)) {
      std_msgs::msg::Bool b; b.data = true; steer_inc_pub_->publish(b);
    }
    if (check_button_press(steer_dec, prev_steer_dec_pressed_)) {
      std_msgs::msg::Bool b; b.data = true; steer_dec_pub_->publish(b);
    }
    if (check_button_press(speed_inc, prev_speed_inc_pressed_)) {
      std_msgs::msg::Bool b; b.data = true; speed_inc_pub_->publish(b);
    }
    if (check_button_press(speed_dec, prev_speed_dec_pressed_)) {
      std_msgs::msg::Bool b; b.data = true; speed_dec_pub_->publish(b);
    }

    bool scale_inc = (msg->buttons.size() > 5 && msg->buttons[5] == 1);
    bool scale_dec = (msg->buttons.size() > 4 && msg->buttons[4] == 1);
    if (check_button_press(scale_inc, prev_scale_inc_pressed_)) {
      steer_scale_ = std::round((steer_scale_ + 0.1) * 10.0) / 10.0;
      if (steer_scale_ < 0.1) steer_scale_ = 0.1;
      RCLCPP_INFO(get_logger(), "steer_scale = %.1f", steer_scale_);
    }
    if (check_button_press(scale_dec, prev_scale_dec_pressed_)) {
      steer_scale_ = std::max(steer_scale_ - 0.1, 0.0);
      steer_scale_ = std::round(steer_scale_ * 10.0) / 10.0;
      if (steer_scale_ < 0.1 && steer_scale_ != 0.0) steer_scale_ = 0.1;
      RCLCPP_INFO(get_logger(), "steer_scale = %.1f", steer_scale_);
    }
  }

  void ack_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
  {
    last_autonomy_msg_ = *msg;
  }

  void timer_callback()
  {
    ackermann_msgs::msg::AckermannDriveStamped out;
    rclcpp::Time current_time = this->get_clock()->now();

    out.header.stamp = current_time;
    out.header.frame_id = frame_id_;

    if ((current_time - last_joy_msg_time_).seconds() > joy_timeout_sec_) {
      if (joy_active_ || ack_active_) {
        RCLCPP_WARN(get_logger(), "Joy message timed out! Stopping the vehicle.");
      }
      joy_active_ = false;
      ack_active_ = false;
    }

    if (joy_active_) {
      out.drive.speed          = joy_speed_;
      out.drive.steering_angle = joy_steer_;
    } else if (ack_active_) {
      out.drive = last_autonomy_msg_;
    } else {
      out.drive.speed          = 0.0;
      out.drive.steering_angle = 0.0;
    }
    drive_pub_->publish(out);
  }

  // --- メンバ変数 ---
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr           joy_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                steer_inc_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                steer_dec_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                speed_inc_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                speed_dec_pub_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  // パラメータ
  double speed_scale_, steer_scale_;
  int joy_button_index_, ack_button_index_;
  int start_button_index_, stop_button_index_;
  int speed_axis_index_, steer_axis_index_; 
  double timer_hz_;
  double joy_timeout_sec_;
  std::string frame_id_; 

  // 状態
  bool joy_active_, ack_active_;
  double joy_speed_, joy_steer_;
  ackermann_msgs::msg::AckermannDrive last_autonomy_msg_;
  rclcpp::Time last_joy_msg_time_;

  // 連射防止フラグ
  bool prev_start_pressed_, prev_stop_pressed_;
  bool prev_steer_inc_pressed_, prev_steer_dec_pressed_;
  bool prev_speed_inc_pressed_, prev_speed_dec_pressed_;
  bool prev_scale_inc_pressed_, prev_scale_dec_pressed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyManagerNode>());
  rclcpp::shutdown();
  return 0;
}