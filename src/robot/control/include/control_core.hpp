#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);
    geometry_msgs::msg::Twist computeCommand(
    const nav_msgs::msg::Path& path, double robot_x, double robot_y, double robot_yaw);
  
  private:
    rclcpp::Logger logger_;
};

} 

#endif 
