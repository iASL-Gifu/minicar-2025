#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>
#include <iomanip>
#include <mutex>
#include <filesystem> 

using namespace std::chrono_literals;

class BagRecorderNode : public rclcpp::Node
{
public:
  BagRecorderNode()
  : Node("bag_recorder_node"), is_recording_(false)
  {
    // === パラメータの宣言と取得 ===
    this->declare_parameter<std::string>("output_dir", ".");
    this->declare_parameter<bool>("all_topics", false);
    this->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>());

    output_dir_ = this->get_parameter("output_dir").as_string();
    record_all_ = this->get_parameter("all_topics").as_bool();
    topics_to_record_ = this->get_parameter("topics").get_parameter_value().get<std::vector<std::string>>();

    try {
        std::time_t t = std::time(nullptr);
        std::tm tm = *std::localtime(&t);
        std::stringstream ss;
        ss << output_dir_ << "/session_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
        session_dir_ = ss.str();
        std::filesystem::create_directories(session_dir_);
        RCLCPP_INFO(this->get_logger(), "Session directory created: %s", session_dir_.c_str());
    } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create session directory: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // === トリガー用のトピックを購読 ===
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/rosbag2_recorder/trigger",
      rclcpp::QoS(10).reliable(),
      std::bind(&BagRecorderNode::trigger_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Bag recorder initialized. Waiting for trigger on /rosbag2_recorder/trigger");
  }

private:
  void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (msg->data) {
      if (!is_recording_) {
        RCLCPP_INFO(this->get_logger(), "Start recording trigger received.");
        start_recording();
        is_recording_ = true;
      } else {
        RCLCPP_WARN(this->get_logger(), "Already recording. Ignoring start trigger.");
      }
    } else {
      if (is_recording_) {
        RCLCPP_INFO(this->get_logger(), "Stop recording trigger received.");
        stop_recording();
        is_recording_ = false;
      } else {
        RCLCPP_WARN(this->get_logger(), "Not currently recording. Ignoring stop trigger.");
      }
    }
  }

  void start_recording()
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::stringstream ss;
    ss << session_dir_ << "/rosbag_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = ss.str();
    storage_options.storage_id = "mcap";

    try {
      writer_->open(storage_options);
      RCLCPP_INFO(this->get_logger(), "Recording to %s", storage_options.uri.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open bag file: %s", e.what());
      is_recording_ = false;
      return;
    }

    if (record_all_) {
        RCLCPP_INFO(this->get_logger(), "Recording all topics.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Recording specified topics.");
    }
    topic_scan_timer_ = this->create_wall_timer(
      1s, std::bind(&BagRecorderNode::discover_and_subscribe_topics, this));
  }

  void stop_recording()
  {
    if (topic_scan_timer_) {
      topic_scan_timer_->cancel();
    }
    subscriptions_.clear();
    subscribed_topics_.clear();
    if (writer_) {
      writer_->close();
      writer_.reset();
      RCLCPP_INFO(this->get_logger(), "Bag file closed.");
    }
  }

  void discover_and_subscribe_topics()
  {
    auto topic_names_and_types = this->get_topic_names_and_types();
    for (const auto & topic_info : topic_names_and_types) {
      const std::string & topic_name = topic_info.first;
      if (subscribed_topics_.count(topic_name)) continue;
      if (!record_all_) {
        bool found = false;
        for(const auto& allowed_topic : topics_to_record_) {
          if (allowed_topic == topic_name) { found = true; break; }
        }
        if (!found) continue;
      }
      if (topic_name.rfind("/parameter_events", 0) == 0 || topic_name.rfind("/rosout", 0) == 0) continue;
      const std::string & topic_type = topic_info.second[0];
      subscribe_to_topic(topic_name, topic_type);
    }
  }

  void subscribe_to_topic(const std::string & topic_name, const std::string & topic_type)
  {
    try {
      rosbag2_storage::TopicMetadata topic_metadata;
      topic_metadata.name = topic_name;
      topic_metadata.type = topic_type;
      topic_metadata.serialization_format = "cdr";
      if(writer_) writer_->create_topic(topic_metadata);
      auto subscription = this->create_generic_subscription(
        topic_name, topic_type, rclcpp::QoS(10),
        [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> message) {
          if (writer_ && is_recording_) {
            auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_message->time_stamp = this->get_clock()->now().nanoseconds();
            bag_message->topic_name = topic_name;
            bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
              new rcutils_uint8_array_t,
              [this](rcutils_uint8_array_t *msg) {
                auto error = rcutils_uint8_array_fini(msg);
                delete msg;
                if (error != RCUTILS_RET_OK) {
                  RCLCPP_ERROR(this->get_logger(), "Failed to destroy serialized message");
                }
              });
            *bag_message->serialized_data = message->get_rcl_serialized_message();
            writer_->write(bag_message);
          }
        });
      subscriptions_.push_back(subscription);
      subscribed_topics_.insert(topic_name);
      RCLCPP_INFO(this->get_logger(), "Subscribed to topic '%s' [%s]", topic_name.c_str(), topic_type.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to topic '%s': %s", topic_name.c_str(), e.what());
    }
  }

  // メンバ変数
  std::mutex mutex_;
  bool is_recording_;
  std::string session_dir_; 

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::unordered_set<std::string> subscribed_topics_;
  rclcpp::TimerBase::SharedPtr topic_scan_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  
  std::string output_dir_;
  bool record_all_;
  std::vector<std::string> topics_to_record_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagRecorderNode>());
  rclcpp::shutdown();
  return 0;
}