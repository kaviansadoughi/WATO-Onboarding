#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    std::vector<int> findPath(const nav_msgs::msg::OccupancyGrid& map, int start_index, int goal_index);

  private:
    rclcpp::Logger logger_;
};

}  

#endif  
