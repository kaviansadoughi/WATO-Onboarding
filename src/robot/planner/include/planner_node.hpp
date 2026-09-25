#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "planner_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();
    void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr odom);
    void goalCallback(geometry_msgs::msg::PointStamped::SharedPtr goal);
    void planPath();

  private:
    robot::PlannerCore planner_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    geometry_msgs::msg::PointStamped::SharedPtr latest_goal_;
};

#endif 
