/**
 * @file test.cpp
 * @author Datta Lohith Gannavarapu, Dheeraj Vishnubhotla, Nazrin Gurbanova
 * @brief This file contains unit tests for a simple ROS 2 node that tests the creation of a publisher.
 * @version 2.0
 * @date 2024-11-24
 * @copyright Copyright (c) 2024
 *
 * This file uses Google Test (gtest) to test a basic ROS 2 node implementation. It focuses on the
 * creation of a publisher and verifies that it has been successfully created by checking the
 * number of publishers associated with a topic.
 */
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class TestNode
 * @brief Unit test class for testing the ROS 2 node functionality.
 * 
 * This class derives from Google Test's Test class and provides the setup for testing a basic
 * ROS 2 node. It initializes the node in the SetUp method, which is executed before each test.
 */
class TestNode : public ::testing::Test {
 protected:
 /**
     * @brief Set up the test environment.
     * 
     * Initializes the ROS 2 node for testing before each test case runs.
     */
    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("test_node");
    }

    rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Test case to verify that a publisher is created successfully.
 * 
 * This test checks that when a publisher is created on the "topic", the number of publishers
 * associated with the topic is equal to 1, indicating successful publisher creation.
 */
TEST_F(TestNode, test_for_publisher) {
    auto publisher = node_->create_publisher<std_msgs::msg::String>
        ("topic", 10);
    auto publishers_number = node_->count_publishers("topic");
    EXPECT_EQ(1, static_cast<int>(publishers_number));
}

/**
 * @brief Test case to verify that no publishers exist initially.
 * 
 * This test checks that before any publisher is created, the number of publishers associated
 * with the "topic" is equal to 0.
 */
TEST_F(TestNode, test_no_initial_publishers) {
    auto publishers_number = node_->count_publishers("topic");
    EXPECT_EQ(0, static_cast<int>(publishers_number));
}

/**
 * @brief Main function to run all tests.
 * 
 * This function initializes the ROS 2 system, runs all the tests defined in the Google Test framework,
 * and then shuts down the ROS 2 system. It returns the result of the test execution.
 *
 * @param argc The number of command-line arguments.
 * @param argv The command-line arguments.
 * @return The result of running the tests.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
