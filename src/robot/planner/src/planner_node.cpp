#include "planner_node.hpp"
#include <cmath>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr odom) {
        this->odomCallback(odom);
    });
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local(), [this](nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        this->mapCallback(map);
    });
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, [this](geometry_msgs::msg::PointStamped::SharedPtr goal) {
        this->goalCallback(goal);
    });
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
}

void PlannerNode::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    latest_map_ = map;
    planPath();
}

void PlannerNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr odom) {
    latest_odom_ = odom;
}

void PlannerNode::goalCallback(geometry_msgs::msg::PointStamped::SharedPtr goal) {
  RCLCPP_INFO(this->get_logger(), "Goal received");
  latest_goal_ = goal;
  planPath();
}

void PlannerNode::planPath() {
  if (!latest_map_ || !latest_odom_ || !latest_goal_) {
      return;
  }

  const auto& q = latest_odom_->pose.pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  double start_x = latest_odom_->pose.pose.position.x - 1.3 * std::cos(yaw);
  double start_y = latest_odom_->pose.pose.position.y - 1.3 * std::sin(yaw);
  double goal_x = latest_goal_->point.x;
  double goal_y = latest_goal_->point.y;

  int start_col = std::floor((start_x - latest_map_->info.origin.position.x) / latest_map_->info.resolution);
  int start_row = std::floor((start_y - latest_map_->info.origin.position.y) / latest_map_->info.resolution);
  int goal_col = std::floor((goal_x - latest_map_->info.origin.position.x) / latest_map_->info.resolution);
  int goal_row = std::floor((goal_y - latest_map_->info.origin.position.y) / latest_map_->info.resolution);

  if (start_col < 0 || start_col >= latest_map_->info.width || start_row < 0 || start_row >= latest_map_->info.height) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = latest_map_->header.frame_id;
    empty_path.header.stamp = this->now();
    path_pub_->publish(empty_path);
    return;
  }
  if (goal_col < 0 || goal_col >= latest_map_->info.width || goal_row < 0 || goal_row >= latest_map_->info.height) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = latest_map_->header.frame_id;
    empty_path.header.stamp = this->now();
    path_pub_->publish(empty_path);
    return;
  }
  size_t start_index = start_row * latest_map_->info.width + start_col;
  size_t goal_index = goal_row * latest_map_->info.width + goal_col;

  int start_cost = latest_map_->data[start_index];
  int goal_cost = latest_map_->data[goal_index];

  RCLCPP_INFO(this->get_logger(), "Start: x=%.2f y=%.2f row=%d col=%d", start_x, start_y, start_row, start_col);

  RCLCPP_INFO(this->get_logger(), "Endpoint costs: start=%d goal=%d", start_cost, goal_cost);

  if (start_cost == -1 || start_cost >= 15 || goal_cost == -1 || goal_cost >= 15 ) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = latest_map_->header.frame_id;
    empty_path.header.stamp = this->now();
    path_pub_->publish(empty_path);
    return;
  }

  auto path_indices = planner_.findPath(*latest_map_, start_index, goal_index);
  RCLCPP_INFO(this->get_logger(), "Path contains %zu cells", path_indices.size());

  nav_msgs::msg::Path path_message;
  path_message.header.frame_id = latest_map_->header.frame_id;
  path_message.header.stamp = this->now();
  
  int width = static_cast<int>(latest_map_->info.width);

  for (int cell_index : path_indices) {
    int path_row = cell_index / width;
    int path_col = cell_index % width;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_message.header;
    pose.pose.position.x = latest_map_->info.origin.position.x + (path_col + 0.5) * latest_map_->info.resolution;
    pose.pose.position.y = latest_map_->info.origin.position.y + (path_row + 0.5) * latest_map_->info.resolution;
    pose.pose.orientation.w = 1.0;
    path_message.poses.push_back(pose);
}
path_pub_->publish(path_message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
