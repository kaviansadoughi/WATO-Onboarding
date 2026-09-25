#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <cstdint>
 
#include "costmap_node.hpp"

 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {
        this->scanCallback(scan);
    });
}

void CostmapNode::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // RCLCPP_INFO(this->get_logger(), "Received %zu ranges", scan->ranges.size());
    int count = 0;
    int column = 0;
    int row = 0;
    int index = 0;
    const double origin_x = -5.0;
    const double origin_y = -5.0;
    const double resolution = 0.1;
    const double inflation_radius = 2.0;
    const int inflation_cells = static_cast<int>(std::ceil(inflation_radius / resolution));
    std::vector<int8_t> costmap(100 * 100, -1);
    for (size_t i = 0; i < scan->ranges.size(); i++) {
      if (std::isfinite(scan->ranges[i]) && scan->ranges[i] >= scan->range_min && scan->ranges[i] <= scan->range_max) {
        count++;
        double angle = scan->angle_min + i * scan->angle_increment;
        double x = scan->ranges[i] * std::cos(angle);
        double y = scan->ranges[i] * std::sin(angle);
        column = std::floor((x - origin_x) / resolution);
        row = std::floor((y - origin_y) / resolution);
        for (double beam_distance = scan->range_min; beam_distance < scan->ranges[i]; beam_distance += resolution / 2.0) {
          double sample_x = beam_distance * std::cos(angle);
          double sample_y = beam_distance * std::sin(angle);
          int sample_col = static_cast<int>(std::floor((sample_x - origin_x) / resolution));
          int sample_row = static_cast<int>(std::floor((sample_y - origin_y) / resolution));

          if (sample_col < 0 || sample_col >= 100 || sample_row < 0 || sample_row >= 100) {
            break;
          }
          if (sample_col == column && sample_row == row) {
            break;
          }
          int sample_index = sample_row * 100 + sample_col;

          if (costmap[sample_index] == -1) {
            costmap[sample_index] = 0;
          }
        }
        if (column >= 0 && column <= 99 && row >= 0 && row <= 99) {
          index = row * 100 + column;
          costmap[index] = 100;
        }
      }
    }
    // RCLCPP_INFO(this->get_logger(), "Valid readings: %d", count);
    const auto obstacle_grid = costmap;
    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 100; j++) {
        if (obstacle_grid[i * 100 + j] == 100) {
          for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
            for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
              int neighbour_row = i + dy;
              int neighbour_col = j + dx;
              if (neighbour_row >= 0 && neighbour_row <= 99 && neighbour_col >= 0 && neighbour_col <= 99) {
                double distance = std::sqrt(dy * dy + dx * dx) * resolution;
                if (distance <= inflation_radius) {
                int cost = static_cast<int>(100 * (1 - distance / inflation_radius));
                int neighbour_index = neighbour_row * 100 + neighbour_col;
                if (costmap[neighbour_index] != -1 && cost > costmap[neighbour_index]) {
                    costmap[neighbour_index] = cost;
                  }
                }
              }
            }
          }
        }
      }
    }
    nav_msgs::msg::OccupancyGrid message;
    message.header = scan->header;
    message.info.resolution = resolution;
    message.info.width = 100;
    message.info.height = 100;
    message.info.origin.position.x = origin_x;
    message.info.origin.position.y = origin_y;
    message.info.origin.orientation.w = 1.0;
    message.data = costmap;
    costmap_pub_->publish(message);
}
 
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}