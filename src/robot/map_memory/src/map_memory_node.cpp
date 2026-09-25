#include "map_memory_node.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <cstdio>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, [this](nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
        this->costMapCallback(costmap);
    });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr odom) {
        this->odomCallback(odom);
    });
    update_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
        this->updateMap();
    });
    global_map_.header.frame_id = "sim_world";
    global_map_.info.resolution = 0.1;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.info.width = 400;
    global_map_.info.height = 400;
    global_map_.info.origin.position.x = -20.0;
    global_map_.info.origin.position.y = -20.0;
    global_map_.data.assign(400 * 400, -1);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());
    loadMap();
    map_pub_->publish(global_map_);
}

void MapMemoryNode::costMapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
    // RCLCPP_INFO(this->get_logger(), "Received grid with %zu cells", costmap->data.size());
    latest_costmap_ = costmap;
}

void MapMemoryNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr odom) {
    // RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f", odom->pose.pose.position.x, odom->pose.pose.position.y);
    latest_odom_ =  odom;
}

void MapMemoryNode::updateMap()
{
    if (!latest_costmap_ || !latest_odom_) {
        return;
    }

    const auto& local = *latest_costmap_;

    double lidar_x = latest_odom_->pose.pose.position.x;
    double lidar_y = latest_odom_->pose.pose.position.y;
    const auto& q = latest_odom_->pose.pose.orientation;

    double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    double dx = lidar_x - last_update_x_;
    double dy = lidar_y - last_update_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (has_updated_map_ && distance < 1.5) {
        return;
    }

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

  for (size_t row = 0; row < global_map_.info.height; ++row) {
      for (size_t col = 0; col < global_map_.info.width; ++col) {
        double world_x = global_map_.info.origin.position.x + (col + 0.5) * global_map_.info.resolution;
        double world_y = global_map_.info.origin.position.y + (row + 0.5) * global_map_.info.resolution;
        double offset_x = world_x - lidar_x;
        double offset_y = world_y - lidar_y;
        double local_x = cos_yaw * offset_x + sin_yaw * offset_y;
        double local_y = -sin_yaw * offset_x + cos_yaw * offset_y;
        int local_col = static_cast<int>(std::floor((local_x - local.info.origin.position.x) / local.info.resolution));
        int local_row = static_cast<int>(std::floor((local_y - local.info.origin.position.y) / local.info.resolution));

        if (local_col < 0 || local_row < 0 || local_col >= static_cast<int>(local.info.width) || local_row >= static_cast<int>(local.info.height)) {
            continue;
        }
        
        size_t local_index = static_cast<size_t>(local_row) * local.info.width + local_col;
        int incoming_cost = local.data[local_index];
        if (incoming_cost == -1) {
            continue;
        }

        size_t global_index = row * global_map_.info.width + col;
        int stored_cost = global_map_.data[global_index];

        if (stored_cost == -1) {
          global_map_.data[global_index] = incoming_cost;
        } else {
          global_map_.data[global_index] = std::max(stored_cost, incoming_cost);
          }
      }
    }

    global_map_.header.stamp = latest_odom_->header.stamp;
    map_pub_->publish(global_map_);

    last_update_x_ = lidar_x;
    last_update_y_ = lidar_y;
    has_updated_map_ = true;
    saveMap();
}

void MapMemoryNode::saveMap()
{
    const char* temporary = "/maps/global_map.tmp";
    const char* destination = "/maps/global_map.txt";

    std::ofstream file(temporary);
    if (!file) {
        RCLCPP_WARN(this->get_logger(), "Could not open map file for saving");
        return;
    }

    file.precision(17);
    file << "WATO_MAP_V1\n"
         << global_map_.header.frame_id << '\n'
         << global_map_.info.width << ' '
         << global_map_.info.height << ' '
         << global_map_.info.resolution << ' '
         << global_map_.info.origin.position.x << ' '
         << global_map_.info.origin.position.y << '\n';

    for (int value : global_map_.data) {
        file << value << ' ';
    }

    file.close();

    if (!file || std::rename(temporary, destination) != 0) {
        RCLCPP_WARN(this->get_logger(), "Map save failed");
    }
}

void MapMemoryNode::loadMap() {
    std::ifstream file("/maps/global_map.txt");
    if (!file) {
        RCLCPP_INFO(this->get_logger(), "No saved map; starting fresh");
        return;
    }

    std::string version;
    std::string frame;
    unsigned int width, height;
    double resolution, origin_x, origin_y;

    if (!(file >> version >> frame >> width >> height >> resolution >> origin_x >> origin_y) || version != "WATO_MAP_V1" ||
        frame != global_map_.header.frame_id ||
        width != global_map_.info.width ||
        height != global_map_.info.height ||
        std::abs(resolution - global_map_.info.resolution) > 1e-6 ||
        std::abs(origin_x - global_map_.info.origin.position.x) > 1e-6 ||
        std::abs(origin_y - global_map_.info.origin.position.y) > 1e-6) {
        RCLCPP_WARN(this->get_logger(), "Saved map settings do not match");
        return;
    }

    auto loaded_data = global_map_.data;

    for (auto& cell : loaded_data) {
        int value;
        if (!(file >> value) || value < -1 || value > 100) {
            RCLCPP_WARN(this->get_logger(), "Saved map data is invalid");
            return;
        }
        cell = static_cast<int8_t>(value);
    }

    global_map_.data = std::move(loaded_data);
    RCLCPP_INFO(this->get_logger(), "Loaded saved map");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}