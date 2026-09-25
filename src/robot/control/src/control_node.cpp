#include "control_node.hpp"
#include <chrono>
#include <cmath>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, [this](nav_msgs::msg::Path::SharedPtr path) {
        this->pathCallback(path);
    });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr odom) {
        this->odomCallback(odom);
    });

cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { 
  this->updateControl(); 
  });
}

void ControlNode::pathCallback(nav_msgs::msg::Path::SharedPtr path) {
    latest_path_ = path;
    RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", path->poses.size());
}

void ControlNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr odom) {
    latest_odom_ = odom;
}

void ControlNode::updateControl() {
    if (!latest_path_ || !latest_odom_ || latest_path_->poses.empty()) {
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
        return;
    }

  const auto& position = latest_odom_->pose.pose.position;
  const auto& q = latest_odom_->pose.pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  double robot_x = position.x - 1.3 * std::cos(yaw);
  double robot_y = position.y - 1.3 * std::sin(yaw);
  const auto& goal = latest_path_->poses.back().pose.position;
  double goal_distance = std::hypot(goal.x - robot_x, goal.y - robot_y);

    if (goal_distance < 0.2) {
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
        return;
    }

    auto command = control_.computeCommand(
    *latest_path_, robot_x, robot_y, yaw);
    cmd_pub_->publish(command);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
