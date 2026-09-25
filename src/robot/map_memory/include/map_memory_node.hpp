#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void costMapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr odom);
    void updateMap();
    void saveMap();
    void loadMap();

  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    nav_msgs::msg::OccupancyGrid global_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    bool has_updated_map_ = false;
    double last_update_x_ = 0.0;
    double last_update_y_ = 0.0;
};

#endif 
