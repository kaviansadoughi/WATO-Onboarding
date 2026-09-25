#include "control_core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

  geometry_msgs::msg::Twist ControlCore::computeCommand(const nav_msgs::msg::Path& path, double robot_x, double robot_y, double robot_yaw) {
    geometry_msgs::msg::Twist command;

    if (path.poses.empty()) {
        return command;
    }

    size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto& point = path.poses[i].pose.position;
        double distance = std::hypot(point.x - robot_x, point.y - robot_y);

        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest = i;
        }
    }

    const double lookahead = 0.6;
    size_t target_index = nearest;
    double accumulated_distance = 0.0;

    while (target_index + 1 < path.poses.size() && accumulated_distance < lookahead) {
        const auto& current = path.poses[target_index].pose.position;
        const auto& next = path.poses[target_index + 1].pose.position;
        accumulated_distance += std::hypot(next.x - current.x, next.y - current.y);
        ++target_index;
    }

    const auto& target = path.poses[target_index].pose.position;
    double dx = target.x - robot_x;
    double dy = target.y - robot_y;
    double target_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    double target_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
    double distance_squared = target_x * target_x + target_y * target_y;

    if (distance_squared < 1e-6) {
        return command;
    }

    double heading_error = std::atan2(target_y, target_x);

    if (std::abs(heading_error) > 0.8) {
        command.angular.z = std::clamp(heading_error, -0.3, 0.3);
        return command;
    }

    double curvature = 2.0 * target_y / distance_squared;
    const auto& goal = path.poses.back().pose.position;
    double goal_distance = std::hypot(goal.x - robot_x, goal.y - robot_y);
    double speed = std::min(0.4, 0.5 * goal_distance);
    speed = std::min(speed, 0.3 / std::max(std::abs(curvature), 1e-6));
    command.linear.x = speed;
    command.angular.z = speed * curvature;
    return command;
  }
}  
