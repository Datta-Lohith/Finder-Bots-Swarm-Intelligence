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

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;
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
  FinderBots(const std::string &node_name, const std::string &robot_name,
  bool navigate = false, double linear_vel = 1.0, double angular_vel = 1.0) 
  : Node(node_name), robot_name_(robot_name), navigate_active_(navigate),
  linear_speed_(linear_vel), angular_speed_(angular_vel), goal_x_(0.0), goal_y_(0.0),
  obstacle_detected_(false), move_flag_(false) {
    initializeTopicsAndTimers();
    RCLCPP_INFO(this->get_logger(), "FinderBots node initialized for: %s", robot_name_.c_str());

  }
  void setGoal(double x, double y) {
    navigate_active_ = true;
    goal_x_ = x;
    goal_y_ = y;
    RCLCPP_INFO(this->get_logger(), "Goal set to: [%.2f, %.2f]", goal_x_, goal_y_);
  }

 private:
  // Member Variables
  std::string robot_name_;
  bool navigate_active_;
  double linear_speed_, angular_speed_;
  double goal_x_, goal_y_;
  bool obstacle_detected_, move_flag_;
  geometry_msgs::msg::Quaternion orientation_;
  std::pair<double, double> current_position_{0.0, 0.0};

  // ROS Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_flag_publisher_;
  
  // ROS Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr move_flag_subscriber_;
  rclcpp::TimerBase::SharedPtr navigation_timer_;

  // Initialize Topics and Timers
  void initializeTopicsAndTimers() {
    // Topic Names
    std::string cmd_vel_topic = "/" + robot_name_ + "/cmd_vel";
    std::string odometry_topic = "/" + robot_name_ + "/odom";
    std::string image_topic = "/" + robot_name_ + "/camera/image_raw";
    std::string lidar_topic = "/" + robot_name_ + "/scan";

    // Publishers
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    move_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/red_object_detected", 10);

    // Subscribers
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, 10, std::bind(&FinderBots::odometryCallback, this, _1));
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, 10, std::bind(&FinderBots::lidarCallback, this, _1));
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10, std::bind(&FinderBots::imageCallback, this, _1));
    move_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/red_object_detected", 10, std::bind(&FinderBots::moveFlagCallback, this, _1));

    // Timers
    navigation_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&FinderBots::navigationLoop, this));
  }
};
