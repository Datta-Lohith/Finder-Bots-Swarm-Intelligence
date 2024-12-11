/**
 * @file test_lv2.cpp
 * @author Datta Lohith Gannavarapu, Dheeraj Vishnubhotla, Nazrin Gurbanova
 * @brief This file contains a test case for validating the publisher on the "swarm" topic in a ROS 2 node.
 * @version 2.0
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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
 * @brief Test case for validating the publisher on the "/robot_1/cmd_vel" topic.
 * 
 * This test subscribes to the "/robot_1/cmd_vel" topic and ensures that a message is received within the specified duration.
 */
TEST_CASE_METHOD(TestNode, "Validate publisher on topic '/robot_1/cmd_vel'",
  "[finderBots]") {
  // Start the test timer
  auto startTime = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - startTime;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);

  // Shared pointer to store the received message
  geometry_msgs::msg::Twist::SharedPtr message = nullptr;

  // Subscription to the "/robot_1/cmd_vel" topic
  auto subscription = create_subscription<geometry_msgs::msg::Twist>(
    "/robot_1/cmd_vel", 10, [&message](geometry_msgs::msg::Twist::UniquePtr
     msg) {message = std::move(msg);});

  // Loop until a message is received or the timeout is reached
  while (rclcpp::ok() && duration < timeout) {
    rclcpp::spin_some(this->get_node_base_interface());
    duration = rclcpp::Clock().now() - startTime;

    if (message != nullptr) {
      break;
    }
  }

  // Ensure that a message has been received
  REQUIRE(message != nullptr);
}

/**
 * @brief Test case for validating the publisher on the "/robot_5/cmd_vel" topic.
 * 
 * This test subscribes to the "/robot_5/cmd_vel" topic and ensures that a message is received within the specified duration.
 */
TEST_CASE_METHOD(TestNode, "Validate publisher on topic '/robot_5/cmd_vel'",
  "[finderBots]") {
  // Start the test timer
  auto startTime = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - startTime;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);

  // Shared pointer to store the received message
  geometry_msgs::msg::Twist::SharedPtr message = nullptr;

  // Subscription to the "/robot_1/cmd_vel" topic
  auto subscription = create_subscription<geometry_msgs::msg::Twist>(
    "/robot_5/cmd_vel", 10, [&message](geometry_msgs::msg::Twist::UniquePtr
     msg) {message = std::move(msg);});

  // Loop until a message is received or the timeout is reached
  while (rclcpp::ok() && duration < timeout) {
    rclcpp::spin_some(this->get_node_base_interface());
    duration = rclcpp::Clock().now() - startTime;

    if (message != nullptr) {
      break;
    }
  }

  // Ensure that a message has been received
  REQUIRE(message != nullptr);
}

/**
 * @brief Test case for validating the publisher on the "/robot_11/cmd_vel" topic.
 * 
 * This test subscribes to the "/robot_11/cmd_vel" topic and ensures that a message is received within the specified duration.
 */
TEST_CASE_METHOD(TestNode, "Validate publisher on topic '/robot_11/cmd_vel'",
  "[finderBots]") {
  // Start the test timer
  auto startTime = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - startTime;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);

  // Shared pointer to store the received message
  geometry_msgs::msg::Twist::SharedPtr message = nullptr;

  // Subscription to the "/robot_1/cmd_vel" topic
  auto subscription = create_subscription<geometry_msgs::msg::Twist>(
    "/robot_11/cmd_vel", 10, [&message](geometry_msgs::msg::Twist::UniquePtr
     msg) {message = std::move(msg);});

  // Loop until a message is received or the timeout is reached
  while (rclcpp::ok() && duration < timeout) {
    rclcpp::spin_some(this->get_node_base_interface());
    duration = rclcpp::Clock().now() - startTime;

    if (message != nullptr) {
      break;
    }
  }

  // Ensure that a message has been received
  REQUIRE(message != nullptr);
}

/**
 * @brief Test case for running a rosbag and checking if the "swarm" topic is subscribed.
 * 
 * This test plays a rosbag file and checks if the "swarm" topic is subscribed and receives messages.
 */
TEST_CASE_METHOD(TestNode, "Run rosbag and check sub 'red_object_detected'",
  "[finderBots]") {
  // Path to the rosbag file
  std::string rosbag_path = ament_index_cpp::get_package_share_directory
    ("finder_bots") + "/test/rosbag";

  // Command to play the rosbag file
  std::string command = "ros2 bag play " + rosbag_path;


  auto startTime = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - startTime;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);

  bool message = false;

  // Create subscription to the "red_object_detected" topic
  auto subscription = create_subscription<std_msgs::msg::Bool>(
      "/red_object_detected", 10, [&message](std_msgs::msg::Bool::UniquePtr
       msg) {message = msg->data;});

  static bool rosbag_played = false;
  while (rclcpp::ok() && duration < timeout) {
    rclcpp::spin_some(this->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Start playing the rosbag file only once
    if (!rosbag_played) {
      std::system(command.c_str());
      rosbag_played = true;
    }
    duration = rclcpp::Clock().now() - startTime;
    // If a message has been received, exit the loop
    if (message) {
      break;
    }
  }
  // Ensure that a message has been received
  REQUIRE(message);
}
