#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TestNode : public ::testing::Test {
protected:
    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("test_node");
    }

    rclcpp::Node::SharedPtr node_;
};

TEST_F(TestNode, test_for_publisher) {
    auto publisher = node_->create_publisher<std_msgs::msg::String>("topic", 10);
    auto publishers_number = node_->count_publishers("topic");
    EXPECT_EQ(1, static_cast<int>(publishers_number));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
