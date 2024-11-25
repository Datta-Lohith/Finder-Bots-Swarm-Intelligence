/**
 * @file test_lv2.cpp
 * @author Datta Lohith Gannavarapu, Dheeraj Vishnubhotla, Nazrin Gurbanova
 * @brief This file contains a test case for validating the publisher on the "swarm" topic in a ROS 2 node.
 * @version 1.0
 * @date 2024-11-24
 * @copyright Copyright (c) 2024
 *
 * This file uses Catch2 to test the functionality of subscribing to a topic and validating the 
 * message received within a specified duration. The test ensures that the "swarm" topic has a publisher 
 * that publishes a message within the given time limit.
 */
#include <catch_amalgamated.hpp>
#include <chrono>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <catch_ros2/catch_ros2.hpp>

/**
 * @class TestNode
 * @brief A ROS 2 test node that subscribes to the "swarm" topic and checks message reception within a time limit.
 * 
 * This class initializes a ROS 2 node that declares a parameter for the test duration, and 
 * subscribes to the "swarm" topic. The class logs the test duration and performs the subscription
 * validation test in its derived test case.
 */
class TestNode : public rclcpp::Node {
 public:
 /**
   * @brief Constructor for the TestNode class.
   * 
   * Declares a parameter "test_duration" to control the duration of the test. The duration is
   * retrieved and logged to ensure the test runs for the specified time.
   */
  TestNode() : Node("test_node") {
    declare_parameter<double>("test_duration", 2.0);
    test_duration_ = get_parameter("test_duration").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Test duration: " << test_duration_);
  }
/**
   * @brief Destructor for the TestNode class.
   * 
   * Logs a message indicating that all tests are completed when the test node is destroyed.
   */
  ~TestNode() {
    RCLCPP_INFO_STREAM(get_logger(), "All tests completed.");
  }
 protected:
  double test_duration_;
};

/**
 * @brief Test case for validating the publisher on the "swarm" topic.
 * 
 * This test subscribes to the "swarm" topic and waits for a message to be received within
 * the specified test duration. The test passes if a message is successfully received within 
 * the time frame; otherwise, it fails.
 */
TEST_CASE_METHOD(TestNode, "Validate publisher on topic 'swarm'",
  "[finderBots]") {
  auto startTime = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - startTime;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);

  std::string message;
  // Create subscription to the "swarm" topic
  auto subscription = create_subscription<std_msgs::msg::String>(
    "swarm", 10, [&message](std_msgs::msg::String::UniquePtr msg) {
      message = msg->data;
    });
  // Loop until a message is received or the timeout is reached
  while (rclcpp::ok() && duration < timeout) {
    rclcpp::spin_some(this->get_node_base_interface());
    duration = rclcpp::Clock().now() - startTime;
    // If a message has been received, exit the loop
    if (!message.empty()) {
      break;
    }
  }
  // Ensure that a message has been received
  REQUIRE_FALSE(message.empty());
}
