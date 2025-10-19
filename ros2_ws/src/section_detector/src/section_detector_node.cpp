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

        // ===== Initialize all member variables =====
        last_x_ = 0.0;
        last_y_ = 0.0;
        pose_x_ = 0.0;
        pose_y_ = 0.0;
        pose_yaw_ = 0.0;
        current_section_ = -1;
        has_valid_params_ = false;
        initialized_ = false;

        // ===== パブリッシャ作成 =====
        section_pub_ = this->create_publisher<std_msgs::msg::Int32>("current_section", 10);

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

            // ===== 初期セクション設定 =====
            current_section_ = -1;
            initialized_ = false;
            RCLCPP_INFO(this->get_logger(), "Initial section set to -1 (will be initialized on first odometry)");

            RCLCPP_INFO(this->get_logger(), "Initialized with %zu circles", circles_.size());
            RCLCPP_INFO(this->get_logger(), "Publish rate: %.2f Hz", publish_rate_hz_);
        }

        // ===== タイマー作成 =====
        if (publish_rate_hz_ > 0.0) {
            auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(period),
                std::bind(&SectionDetectorNode::publish_section, this));
        } else {
            RCLCPP_WARN(this->get_logger(), "publish_rate_hz <= 0.0, timer not started");
        }
    }

private:
    // ===== Pose コールバック（初期化用）=====
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if (!has_valid_params_) return;

        RCLCPP_INFO(this->get_logger(), "=== Pose callback received ===");

        // poseの座標を取得
        double pose_x = msg->pose.pose.position.x;
        double pose_y = msg->pose.pose.position.y;

        RCLCPP_INFO(this->get_logger(),
            "Pose position: (%.2f, %.2f)", pose_x, pose_y);
        RCLCPP_INFO(this->get_logger(),
            "Quaternion: (x=%.4f, y=%.4f, z=%.4f, w=%.4f)",
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        // クォータニオンからヨー角を計算
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pose_yaw_ = -yaw;

        RCLCPP_INFO(this->get_logger(),
            "RPY: (roll=%.4f, pitch=%.4f, yaw=%.4f rad)", roll, pitch, yaw);

        // pose の座標を保存
        pose_x_ = pose_x;
        pose_y_ = pose_y;

        // yaml で設定した点の座標から pose の座標を引いて、相対座標に変換
        circles_relative_ = circles_;  // 元の座標からリセット
        for (size_t i = 0; i < circles_relative_.size(); ++i) {
            // 1. 事前に pose の座標を引く（平行移動）
            double translated_x = circles_relative_[i].x - pose_x;
            double translated_y = circles_relative_[i].y - pose_y;

            // 2. その後に回転
            double rotated_x = translated_x * std::cos(pose_yaw_) - translated_y * std::sin(pose_yaw_);
            double rotated_y = translated_x * std::sin(pose_yaw_) + translated_y * std::cos(pose_yaw_);

            circles_relative_[i].x = rotated_x;
            circles_relative_[i].y = rotated_y;
        }

        RCLCPP_INFO(this->get_logger(),
            "Circles converted to relative coordinates. Pose: (%.2f, %.2f), Yaw: %.4f rad", 
            pose_x, pose_y, pose_yaw_);

        // 現在位置を原点に設定（odomの起点）
        last_x_ = 0.0;
        last_y_ = 0.0;

        find_nearest_section();
        initialized_ = true;
    }

    // ===== Odometryコールバック =====
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!has_valid_params_ || !initialized_) return;
        if (circles_relative_.empty()) return;

        last_x_ = msg->pose.pose.position.x;
        last_y_ = msg->pose.pose.position.y;

        // ===== 次の円番号を計算 =====
        size_t next_idx = (current_section_ + 1) % circles_relative_.size();

        // ===== 次の円までの距離を計算 =====
        double dx = last_x_ - circles_relative_[next_idx].x;
        double dy = last_y_ - circles_relative_[next_idx].y;
        double distance = std::sqrt(dx * dx + dy * dy);

        RCLCPP_INFO(this->get_logger(),
            "Current pos: (%.2f, %.2f), Next circle[%zu]: (%.2f, %.2f), radius: %.2f, Distance: %.2f m",
            last_x_, last_y_, next_idx, circles_relative_[next_idx].x, circles_relative_[next_idx].y,
            circles_relative_[next_idx].radius, distance);

        // ===== 次の円を通過したか確認 =====
        if (is_inside_circle(last_x_, last_y_, circles_relative_[next_idx])) {
            passed_sections_[next_idx] = true;
            current_section_ = static_cast<int>(next_idx);
            RCLCPP_INFO(this->get_logger(),
                "Passed circle %zu → section = %d (x=%.2f, y=%.2f)",
                next_idx, current_section_, last_x_, last_y_);
        }
    }

    // ===== 隣同士の2つの円（円iと円i+1）の距離を計算し、最も距離が小さいペアをセクションとする =====
    void find_nearest_section() {
        if (circles_.empty()) return;

        double min_pair_distance = std::numeric_limits<double>::max();
        int nearest_section = -1;

        // 隣同士の円のペアを全て確認
        for (size_t i = 0; i < circles_.size(); ++i) {
            size_t next_i = (i + 1) % circles_.size();

            // 円iと円(i+1)、そして現在位置との距離を計算
            // 現在位置から円iまでの距離
            double dx1 = pose_x_ - circles_[i].x;
            double dy1 = pose_y_ - circles_[i].y;
            double dist_to_i = std::sqrt(dx1 * dx1 + dy1 * dy1);

            // 現在位置から円(i+1)までの距離
            double dx2 = pose_x_ - circles_[next_i].x;
            double dy2 = pose_y_ - circles_[next_i].y;
            double dist_to_next_i = std::sqrt(dx2 * dx2 + dy2 * dy2);

            // 2つの円までの距離の合計
            double pair_distance = dist_to_i + dist_to_next_i;

            RCLCPP_DEBUG(this->get_logger(),
                "Pair distance (circle[%zu] + circle[%zu]) = %.2f + %.2f = %.2f",
                i, next_i, dist_to_i, dist_to_next_i, pair_distance);

            if (pair_distance < min_pair_distance) {
                min_pair_distance = pair_distance;
                nearest_section = static_cast<int>(i);
            }
        }

        if (nearest_section != -1) {
            current_section_ = nearest_section;
            passed_sections_[nearest_section] = true;
            RCLCPP_INFO(this->get_logger(),
                "Initialized section to %d (nearest pair distance=%.2f, x=%.2f, y=%.2f)",
                current_section_, min_pair_distance, pose_x_, pose_y_);
        }
    }

    // ===== 定期publish =====
    void publish_section() {
        if (!has_valid_params_) return;

        std_msgs::msg::Int32 msg_out;
        msg_out.data = current_section_;
        section_pub_->publish(msg_out);
    }

    bool is_inside_circle(double x, double y, const Circle &circle) {
        double dx = x - circle.x;
        double dy = y - circle.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        return distance <= circle.radius;
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr section_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Circle> circles_, circles_relative_;
    std::vector<bool> passed_sections_;
    double last_x_;
    double last_y_;
    double pose_x_, pose_y_;
    double pose_yaw_;
    int current_section_;
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