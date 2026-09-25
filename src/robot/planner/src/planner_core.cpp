#include "planner_core.hpp"
#include <limits>
#include <queue>
#include <functional>
#include <utility>
#include <cstdlib>
#include <algorithm>
#include <cmath>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {

}

std::vector<int> PlannerCore::findPath(const nav_msgs::msg::OccupancyGrid& map, int start_index, int goal_index) {
    
    size_t cell_count = map.data.size();
    std::vector<double> g_score (cell_count, std::numeric_limits<double>::infinity());
    std::vector<int> came_from (cell_count, -1);
    g_score[start_index] = 0;
    int width = static_cast<int>(map.info.width);
    int start_row = start_index / width;
    int start_col = start_index % width;
    int goal_row = goal_index / width;
    int goal_col = goal_index % width;
    double start_h = std::hypot(goal_row - start_row, goal_col - start_col);

    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> open_set;
    open_set.push({start_h, start_index});

    const int row_offsets[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int col_offsets[8] = {0, 0, -1, 1, -1, 1, -1, 1};

    while (!open_set.empty()) {
    int current_index = open_set.top().second;
    open_set.pop();
    if (current_index == goal_index) {
            std::vector<int> path;
            int trace_index = goal_index;
            while (trace_index != -1) {
                path.push_back(trace_index);
                trace_index = came_from[trace_index];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        int current_row = current_index / width;
        int current_col = current_index % width;
        for (int direction = 0; direction < 8; direction++) {
            int neighbour_row = current_row + row_offsets[direction];
            int neighbour_col = current_col + col_offsets[direction];
            if (neighbour_col >= map.info.width || neighbour_col < 0 || neighbour_row >= map.info.height || neighbour_row < 0) {
                continue;
            }
            bool diagonal = row_offsets[direction] != 0 && col_offsets[direction] != 0;
            if (diagonal) {
                int side_a = map.data[current_row * width + neighbour_col];
                int side_b = map.data[neighbour_row * width + current_col];
                if (side_a == -1 || side_a >= 15 || side_b == -1 || side_b >= 15) {
                    continue;
                }
            }
            int neighbour_index = neighbour_row * width + neighbour_col;
            int neighbour_cost = map.data[neighbour_index];
            if (neighbour_cost == -1 || neighbour_cost >= 15) {
                continue;
            }
            double step_cost = diagonal ? std::sqrt(2.0) : 1.0;
            double tentative_g = g_score[current_index] + step_cost * (1.0 + neighbour_cost / 100.0);
            if (tentative_g < g_score[neighbour_index]) {
                came_from[neighbour_index] =  current_index;
                g_score[neighbour_index] =  tentative_g;
                double goal_h = std::hypot(goal_row - neighbour_row, goal_col - neighbour_col);
                open_set.push({tentative_g + goal_h, neighbour_index});
            }
        }
    }

    return {};  
    }
} 
