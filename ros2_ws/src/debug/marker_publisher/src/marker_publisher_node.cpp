#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>

struct Circle {
  double x;
  double y;
  double radius;
};

class MarkerPublisherNode : public rclcpp::Node
{
public:
  MarkerPublisherNode()
    : Node("marker_publisher")
  {
    // パラメーターの宣言
    this->declare_parameter<std::vector<double>>("circles", std::vector<double>{});
    
    // Markerパブリッシャーの作成
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
    
    // パラメーターから円の情報を取得
    auto circles_param = this->get_parameter("circles").as_double_array();
    
    // パラメーターをパース
    if (circles_param.empty()) {
      RCLCPP_WARN(this->get_logger(), "No circles defined");
      has_valid_params_ = false;
    } else if (circles_param.size() % 3 != 0) {
      RCLCPP_ERROR(this->get_logger(),
        "circles parameter must have a multiple of 3 elements (x, y, radius for each circle)");
      has_valid_params_ = false;
    } else {
      // パラメーターを Circle 構造体にパース
      for (size_t i = 0; i < circles_param.size(); i += 3) {
        Circle circle;
        circle.x = circles_param[i];
        circle.y = circles_param[i + 1];
        circle.radius = circles_param[i + 2];
        circles_.push_back(circle);
      }
      
      has_valid_params_ = true;
      RCLCPP_INFO(this->get_logger(), "Marker Publisher Node started with %zu circles", circles_.size());
      
      // デバッグ出力
      for (size_t i = 0; i < circles_.size(); ++i) {
        RCLCPP_DEBUG(this->get_logger(), 
          "Circle %zu: x=%.2f, y=%.2f, radius=%.2f",
          i, circles_[i].x, circles_[i].y, circles_[i].radius);
      }
    }
    
    // タイマーの作成（1秒ごとにマーカーを発行）
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MarkerPublisherNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (!has_valid_params_) {
      return;
    }

    auto marker_array = visualization_msgs::msg::MarkerArray();
    
    for (size_t idx = 0; idx < circles_.size(); ++idx) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "circles";
      marker.id = idx;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      
      // 位置の設定
      marker.pose.position.x = circles_[idx].x;
      marker.pose.position.y = circles_[idx].y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      
      // サイズの設定（直径 = 2 * radius）
      marker.scale.x = 2 * circles_[idx].radius;
      marker.scale.y = 2 * circles_[idx].radius;
      marker.scale.z = 0.1;
      
      // 色の設定（RGBA）
      marker.color.r = 0.0;
      marker.color.g = 0.5;
      marker.color.b = 1.0;
      marker.color.a = 0.7;
      
      marker_array.markers.push_back(marker);
    }
    
    // マーカー配列を発行
    marker_pub_->publish(marker_array);
    RCLCPP_DEBUG(this->get_logger(), "Published %zu markers", marker_array.markers.size());
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Circle> circles_;
  bool has_valid_params_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisherNode>());
  rclcpp::shutdown();
  return 0;
}