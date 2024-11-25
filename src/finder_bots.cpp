#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using STRING = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;

class RobotSwarm : public rclcpp::Node {
 public:
  RobotSwarm() : Node("RobotSwarm"), count_(0) {
    this->declare_parameter<std::string>("topic_name", "swarm");
    this->declare_parameter<int>("publish_frequency", 500);

    auto topicName = this->get_parameter("topic_name").as_string();
    auto publishFrequency = this->get_parameter("publish_frequency").as_int();

    publisher_ = this->create_publisher<STRING>(topicName, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publishFrequency), std::bind(&RobotSwarm::timer_callback, this));
  }

 private:
  size_t count_;
  PUBLISHER publisher_;
  TIMER timer_;

  void timer_callback() {
    auto message = STRING();
    message.data = "Robot Swarm Node Called " + std::to_string(count_);
    RCLCPP_INFO(this->get_logger(), "Publishing message number %zu: %s", count_, message.data.c_str());
    publisher_->publish(message);
    count_++;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotSwarm>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}