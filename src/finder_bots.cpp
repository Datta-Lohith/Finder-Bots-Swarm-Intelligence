/**
 * @file finder_bots.cpp
 * @author Datta Lohith Gannavarapu, Dheeraj Vishnubhotla, Nazrin Gurbanova
 * @brief This file contains the implementation of the FinderBots class that publishes messages
 *        at regular intervals using ROS 2.
 * @version 2.0
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
  * @brief  for FinderBots class.
  * 
  * Initializes the ROS 2 node, declares parameters for topic name and publish frequency,
  * and sets up the publisher and timer. The timer calls the `timer_callback` function 
  * at the specified frequency.
  */
  FinderBots(const std::string &node_name, const std::string &robot_name,
  bool navigate = false, double linear_vel = 1.0)
  : Node(node_name), robot_name_(robot_name), navigate_active_(navigate),
  linear_speed_(linear_vel), goal_x_(0.0),
  goal_y_(0.0), obstacle_detected_(false), move_flag_(false) {
    initializeTopicsAndTimers();
    RCLCPP_INFO(this->get_logger(), "FinderBots node initialized for: %s",
     robot_name_.c_str());
  }

 /**
  * @brief Sets the goal position for the robot.
  * 
  * Activates navigation and sets the goal coordinates for the robot to move towards.
  * @param x The x-coordinate of the goal position.
  * @param y The y-coordinate of the goal position.
  */
  void setGoal(double x, double y) {
    navigate_active_ = true;
    goal_x_ = x;
    goal_y_ = y;
    RCLCPP_INFO(this->get_logger(), "Goal set to: [%.2f, %.2f]",
     goal_x_, goal_y_);
  }

 private:
  // Member Variables
  std::string robot_name_;
  bool navigate_active_;
  double linear_speed_;
  double goal_x_, goal_y_;
  bool obstacle_detected_, move_flag_;
  geometry_msgs::msg::Quaternion orientation_;
  std::pair<double, double> current_position_{0.0, 0.0};

  // ROS Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_flag_publisher_;

  // ROS Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    lidar_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr move_flag_subscriber_;
  rclcpp::TimerBase::SharedPtr navigation_timer_;

 /**
  * @brief Initializes the topics and timers for the FinderBots class.
  * 
  * Sets up the publishers, subscribers, and timers for the FinderBots class.
  */
  void initializeTopicsAndTimers() {
    // Topic Names
    std::string cmd_vel_topic = "/" + robot_name_ + "/cmd_vel";
    std::string odometry_topic = "/" + robot_name_ + "/odom";
    std::string image_topic = "/" + robot_name_ + "/camera/image_raw";
    std::string lidar_topic = "/" + robot_name_ + "/scan";

    // Publishers
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>
      (cmd_vel_topic, 10);
    move_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>
      ("/red_object_detected", 10);

    // Subscribers
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, 10, std::bind(&FinderBots::odometryCallback, this, _1));
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, 10, std::bind(&FinderBots::lidarCallback, this, _1));
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10, std::bind(&FinderBots::imageCallback, this, _1));
    move_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/red_object_detected", 10,
        std::bind(&FinderBots::moveFlagCallback, this, _1));

    // Timers
    navigation_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
     std::bind(&FinderBots::navigationLoop, this));
  }

  // Callbacks
 /**
  * @brief Callback function for odometry data.
  * 
  * Updates the current position and orientation of the robot based on odometry data.
  * @param msg The odometry message.
  */
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_.first = msg->pose.pose.position.x;
    current_position_.second = msg->pose.pose.position.y;
    orientation_ = msg->pose.pose.orientation;
  }

 /**
  * @brief Callback function for lidar data.
  * 
  * Checks for obstacles in the robot's path using lidar data. If an obstacle is detected
  * within a certain threshold, it sets the obstacle_detected_ flag to true.
  * @param msg The lidar scan message.
  */
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto threshold = 0.5;
    obstacle_detected_ = false;

    // Indices for the front of the robot for 40 degrees on either hemisphere
    int left_start_index = 0;
    int left_end_index = 40;
    int right_start_index = 320;
    int right_end_index = 359;

    auto check_obstacle = [&](int start, int end) {
        for (int i = start; i <= end; ++i) {
            if (msg->ranges[i] < threshold) {
                obstacle_detected_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "Obstacle detected in front. Range: %f at index %d",
                        msg->ranges[i], i);
                return true;
            }
        }
        return false;
    };

    if (!check_obstacle(left_start_index, left_end_index)) {
    check_obstacle(right_start_index, right_end_index);
    }
  }

 /**
  * @brief Callback function for image data.
  * 
  * Processes the image data to detect red objects. If a red object is detected,
  * it sets the move_flag_ to true and publishes a message indicating the detection.
  * @param msg The image message.
  */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,
        sensor_msgs::image_encodings::BGR8);
      cv::Mat hsv_image;
      cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

      cv::Scalar lower_red(0, 100, 100);
      cv::Scalar upper_red(10, 255, 255);

      cv::Mat red_mask;
      cv::inRange(hsv_image, lower_red, upper_red, red_mask);

      if (cv::countNonZero(red_mask) > 15000 && !move_flag_) {
        RCLCPP_INFO(this->get_logger(), "Red object detected!");
        move_flag_ = true;
        std_msgs::msg::Bool move_flag_msg;
        move_flag_msg.data = true;
        move_flag_publisher_->publish(move_flag_msg);
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

 /**
  * @brief Callback function for move flag.
  * 
  * Updates the move_flag_ based on the received message.
  * @param msg The move flag message.
  */
  void moveFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    move_flag_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Move flag updated: %s",
     move_flag_ ? "true" : "false");
  }

  // Navigation Logic
 /**
  * @brief Navigation loop for the FinderBots class.
  * 
  * Implements the navigation logic for the robot. It calculates the distance to the goal
  * and the angle to the goal, and moves the robot accordingly. If an obstacle is detected,
  * it avoids the obstacle by turning slightly.
  */
  void navigationLoop() {
    if (!navigate_active_) return;

    double distance_to_goal = calculateEuclideanDistance(current_position_,
     {goal_x_, goal_y_});

    if (distance_to_goal <= 0.1) {
      stopRobot();
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
    } else if (!move_flag_) {
        if (!obstacle_detected_) {
        double angle_to_goal = calculateAngleToGoal();
        moveRobot(linear_speed_, angle_to_goal);
      } else {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected(Avoiding obstacle)");
        moveRobot(0.0, 0.2);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Red object detected. Halting movement.");
      stopRobot();
    }
  }

  // Helper Methods
 /**
  * @brief Calculates the Euclidean distance between two points.
  * 
  * @param a The first point.
  * @param b The second point.
  * @return The Euclidean distance between the two points.
  */
  double calculateEuclideanDistance(const std::pair<double, double>& a,
                                     const std::pair<double, double>& b) {
    return std::sqrt(std::pow(b.first - a.first, 2) +
     std::pow(b.second - a.second, 2));
  }

 /**
  * @brief Calculates the angle to the goal position.
  * 
  * @return The angle to the goal position.
  */
  double calculateAngleToGoal() {
    double angle_to_goal = std::atan2(goal_y_ - current_position_.second,
                                      goal_x_ - current_position_.first);
    return normalizeAngle(angle_to_goal - getYaw());
  }

 /**
  * @brief Gets the yaw angle from the orientation quaternion.
  */
  double getYaw() {
    tf2::Quaternion quaternion(orientation_.x, orientation_.y,
     orientation_.z, orientation_.w);
    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
  }

 /**
  * @brief Normalizes the angle to the range [-pi, pi].
  * 
  * @param angle The angle to normalize.
  * @return The normalized angle.
  */
  double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }

 /**
  * @brief Moves the robot with the specified linear and angular velocities.
  * 
  * @param linear The linear velocity.
  * @param angular The angular velocity.
  */
  void moveRobot(double linear, double angular) {
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = linear;
    cmd_msg.angular.z = angular;
    velocity_publisher_->publish(cmd_msg);
  }

 /**
  * @brief Stops the robot by setting the linear and angular velocities to zero.
  */
  void stopRobot() {
    navigate_active_ = false;
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 0.0;
    velocity_publisher_->publish(cmd_msg);
  }
};
