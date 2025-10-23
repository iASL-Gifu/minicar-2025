#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <cmath>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

struct Circle {
  double x;
  double y;
  double radius;
};

class SectionDetectorNode : public rclcpp::Node {
public:
  SectionDetectorNode() : Node("section_detector") {
    // ===== パラメータ宣言 =====
    this->declare_parameter<std::vector<double>>("circles", std::vector<double>{});
    this->declare_parameter<double>("publish_rate_hz", 1.0);

    // ===== 変数初期化 =====
    last_x_ = 0.0;
    last_y_ = 0.0;
    pose_x_ = 0.0;
    pose_y_ = 0.0;
    pose_yaw_ = 0.0;
    current_section_ = -1;
    previous_section_ = -1;
    loop_count_ = 0;
    has_valid_params_ = false;
    initialized_ = false;

    // ===== パブリッシャ作成 =====
    section_pub_ = this->create_publisher<std_msgs::msg::Int32>("current_section", 10);
    loop_pub_ = this->create_publisher<std_msgs::msg::Int32>("loop_count", 10);  // ← 追加

    // ===== サブスクライバ作成 =====
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/visual_localization/pose", 10,
      std::bind(&SectionDetectorNode::pose_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/visual_slam/tracking/odometry", 10,
      std::bind(&SectionDetectorNode::odom_callback, this, std::placeholders::_1));

    // ===== パラメータ取得 =====
    auto circles_param = this->get_parameter("circles").as_double_array();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    if (circles_param.empty()) {
      RCLCPP_WARN(this->get_logger(), "No circles defined");
      has_valid_params_ = false;
    } else if (circles_param.size() % 3 != 0) {
      RCLCPP_ERROR(this->get_logger(),
        "circles parameter must have a multiple of 3 elements (x, y, radius for each circle)");
      has_valid_params_ = false;
    } else {
      for (size_t i = 0; i < circles_param.size(); i += 3) {
        Circle c;
        c.x = circles_param[i];
        c.y = circles_param[i + 1];
        c.radius = circles_param[i + 2];
        circles_.push_back(c);
      }
      has_valid_params_ = true;
      passed_sections_.resize(circles_.size(), false);
      circles_relative_ = circles_;
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initialized with %zu circles", circles_.size());
    }

    // ===== タイマー作成 =====
    if (publish_rate_hz_ > 0.0) {
      auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&SectionDetectorNode::publish_data, this));  // ← section と loop 両方pub
    } else {
      RCLCPP_WARN(this->get_logger(), "publish_rate_hz <= 0.0, timer not started");
    }
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!has_valid_params_) return;
    RCLCPP_INFO(this->get_logger(), "=== Pose callback received ===");

    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose_yaw_ = -yaw;

    // 座標変換
    circles_relative_ = circles_;
    for (size_t i = 0; i < circles_relative_.size(); ++i) {
      double tx = circles_relative_[i].x - pose_x_;
      double ty = circles_relative_[i].y - pose_y_;
      circles_relative_[i].x = tx * std::cos(pose_yaw_) - ty * std::sin(pose_yaw_);
      circles_relative_[i].y = tx * std::sin(pose_yaw_) + ty * std::cos(pose_yaw_);
    }

    last_x_ = 0.0;
    last_y_ = 0.0;
    find_nearest_section();
    initialized_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!has_valid_params_ || !initialized_ || circles_relative_.empty()) return;

    last_x_ = msg->pose.pose.position.x;
    last_y_ = msg->pose.pose.position.y;

    size_t next_idx = (current_section_ + 1) % circles_relative_.size();
    if (is_inside_circle(last_x_, last_y_, circles_relative_[next_idx])) {
      passed_sections_[next_idx] = true;
      previous_section_ = current_section_;
      current_section_ = static_cast<int>(next_idx);

      // === section0通過判定 ===
      if (current_section_ == 0 && previous_section_ != 0) {
        loop_count_++;
        RCLCPP_INFO(this->get_logger(), "Loop %d completed!", loop_count_);
      }

      // === 3周したら current_section = 10 に固定 ===
      if (loop_count_ >= 4) {
        current_section_ = 10;
        RCLCPP_WARN(this->get_logger(),
          "Loop count reached 3, section forced to 10.");
      }

      RCLCPP_INFO(this->get_logger(),
        "Passed circle %zu → section = %d (x=%.2f, y=%.2f)",
        next_idx, current_section_, last_x_, last_y_);
    }
  }

  void find_nearest_section() {
    if (circles_.empty()) return;
    double min_pair_distance = std::numeric_limits<double>::max();
    int nearest_section = -1;

    for (size_t i = 0; i < circles_.size(); ++i) {
      size_t next_i = (i + 1) % circles_.size();
      double dist_sum =
        std::hypot(pose_x_ - circles_[i].x, pose_y_ - circles_[i].y) +
        std::hypot(pose_x_ - circles_[next_i].x, pose_y_ - circles_[next_i].y);
      if (dist_sum < min_pair_distance) {
        min_pair_distance = dist_sum;
        nearest_section = static_cast<int>(i);
      }
    }

    if (nearest_section != -1) {
      current_section_ = nearest_section;
      passed_sections_[nearest_section] = true;
      RCLCPP_INFO(this->get_logger(),
        "Initialized section to %d (pair_distance=%.2f)", nearest_section, min_pair_distance);
    }
  }

  void publish_data() {
    if (!has_valid_params_) return;

    std_msgs::msg::Int32 msg_section;
    msg_section.data = current_section_;
    section_pub_->publish(msg_section);

    std_msgs::msg::Int32 msg_loop;
    msg_loop.data = loop_count_;
    loop_pub_->publish(msg_loop);
  }

  bool is_inside_circle(double x, double y, const Circle &circle) {
    return std::hypot(x - circle.x, y - circle.y) <= circle.radius;
  }

  // ===== メンバ変数 =====
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr section_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr loop_pub_;  // ← 追加
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Circle> circles_, circles_relative_;
  std::vector<bool> passed_sections_;

  double last_x_, last_y_;
  double pose_x_, pose_y_, pose_yaw_;
  int current_section_;
  int previous_section_;
  int loop_count_;
  double publish_rate_hz_;
  bool has_valid_params_;
  bool initialized_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SectionDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
