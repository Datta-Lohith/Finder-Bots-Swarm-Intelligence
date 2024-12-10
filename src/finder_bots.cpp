/**
 * @file finder_bots.cpp
 * @author Datta Lohith Gannavarapu, Dheeraj Vishnubhotla, Nazrin Gurbanova
 * @brief This file contains the implementation of the FinderBots class that publishes messages
 *        at regular intervals using ROS 2.
 * @version 1.0
 * @date 2024-11-24
 * @copyright Copyright (c) 2024
 *
 * This file provides the implementation of the FinderBots class, which creates a ROS 2
 * publisher to publish messages with a topic name defined by the user and a frequency 
 * specified via parameters. It uses a timer to periodically call a callback function
 * that publishes a message with an incrementing count.
 */

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Define aliases for convenience
using STRING = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;

/**
 * @class FinderBots
 * @brief A ROS 2 Node that publishes a message with an incrementing count at regular intervals.
 * 
 * The FinderBots class initializes a ROS 2 node that publishes messages at a user-defined
 * frequency. The messages contain an incrementing count to indicate the number of times
 * the node has published. The frequency and topic name are configurable parameters.
 */
class FinderBots : public rclcpp::Node {
 public:
 /**
   * @brief Constructor for FinderBots class.
   * 
   * Initializes the ROS 2 node, declares parameters for topic name and publish frequency,
   * and sets up the publisher and timer. The timer calls the `timer_callback` function 
   * at the specified frequency.
   */
  FinderBots() : Node("FinderBots"), count_(0) {
    // Declare parameters for topic name and publish frequency
    this->declare_parameter<std::string>("topic_name", "swarm");
    this->declare_parameter<int>("publish_frequency", 500);
    // Get parameter values
    auto topicName = this->get_parameter("topic_name").as_string();
    auto publishFrequency = this->get_parameter("publish_frequency").as_int();
    // Create a publisher and a timer for periodic callback
    publisher_ = this->create_publisher<STRING>(topicName, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publishFrequency),
          std::bind(&FinderBots::timer_callback, this));
  }

 private:
  size_t count_;
  PUBLISHER publisher_;
  TIMER timer_;

  /**
     * @brief Callback function called by the timer to publish messages.
     * 
     * The callback generates a new message with an incrementing count, logs the publishing
     * event, and publishes the message to the specified topic.
     */
  void timer_callback() {
    auto message = STRING();
    message.data = "Robot Swarm Node Called " + std::to_string(count_);
    // Log the publishing event
    RCLCPP_INFO(this->get_logger(), "Publishing message number %zu: %s",
      count_, message.data.c_str());
    publisher_->publish(message);
    // Increment the count for the next message
    count_++;
  }
};

/**
 * @brief Main function to initialize, spin, and shutdown the ROS 2 node.
 * 
 * Initializes the ROS 2 system, creates an instance of the FinderBots node, and starts
 * the event loop. After the event loop is finished, the ROS 2 system is shut down.
 *
 * @param argc The number of command-line arguments.
 * @param argv The command-line arguments.
 * @return 0 on successful execution.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FinderBots>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
