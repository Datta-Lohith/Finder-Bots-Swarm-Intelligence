#include <catch_amalgamated.hpp>
#include <chrono>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <catch_ros2/catch_ros2.hpp>

class TestNode : public rclcpp::Node {
 public:
  TestNode() : Node("test_node") {
    declare_parameter<double>("test_duration", 2.0);
    test_duration_ = get_parameter("test_duration").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Test duration: " << test_duration_);
  }

  ~TestNode() {
    RCLCPP_INFO_STREAM(get_logger(), "All tests completed.");
  }
 protected:
  double test_duration_;
};

TEST_CASE_METHOD(TestNode, "Validate publisher on topic 'swarm'",
  "[finderBots]") {
  auto startTime = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - startTime;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);

  std::string message;

  auto subscription = create_subscription<std_msgs::msg::String>(
    "swarm", 10, [&message](std_msgs::msg::String::UniquePtr msg) {
      message = msg->data;
    });
  while (rclcpp::ok() && duration < timeout) {
    rclcpp::spin_some(this->get_node_base_interface());
    duration = rclcpp::Clock().now() - startTime;

    if (!message.empty()) {
      break;
    }
  }

  REQUIRE_FALSE(message.empty());
}