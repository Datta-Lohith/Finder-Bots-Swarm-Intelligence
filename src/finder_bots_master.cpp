#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "finder_bots.cpp"

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;

class FinderBotsMaster : public rclcpp::Node {
 private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::shared_ptr<FinderBots>> robots_;
  int robot_count_;

 public:
  FinderBotsMaster(const std::vector<std::shared_ptr<FinderBots>> &robots, int robot_count)
      : Node("master_node"), robots_(robots), robot_count_(robot_count) {
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&FinderBotsMaster::controlLoop, this));    
    initializeGoals(10.0);
  }

  void controlLoop() {
    RCLCPP_INFO(this->get_logger(), "Iterating through the swarm...");
  }

  void initializeGoals(double distance) {
    for (int i = 0; i < robot_count_; ++i) {
      double offset = static_cast<double>(i - (robot_count_ / 2));
      robots_[i]->setGoal(distance, offset);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  int nodes = 12;

  std::vector<std::shared_ptr<FinderBots>> robot_array;

  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the distribution for real numbers between 0 and 1
  std::uniform_real_distribution<double> distribution(0.1, 0.2);

  // Generate a random number
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    double random_number = distribution(gen);
    auto robot = std::make_shared<FinderBots>(nodename, r_namespace, false, random_number);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  auto node =
      std::make_shared<FinderBotsMaster>(robot_array, static_cast<int>(nodes));
  exec.add_node(node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}