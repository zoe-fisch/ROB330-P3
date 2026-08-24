#pragma once

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include "obstacle_distance_grid.hpp"

#include <vector>
#include <unordered_map>
#include <queue>

struct ThetaStarNode {
    int x, y;
    float cost;      // g-cost
    float priority;  // f-cost = g + h
    bool operator>(const ThetaStarNode& other) const {
        return priority > other.priority;
    } // Min-Priority Queue sorted by priority
};

class ThetaStarPlanner {
public:
    ThetaStarPlanner() = default;

    bool planPath(const ObstacleDistanceGrid& dist_grid,
                  const geometry_msgs::msg::Pose2D& start,
                  const geometry_msgs::msg::Pose2D& goal,
                  mbot_interfaces::msg::Pose2DArray& path);

private:
    float heuristic(int x1, int y1, int x2, int y2);

    std::vector<std::pair<int, int>> backtraceWaypoints(
        const std::unordered_map<int, int>& came_from, int goal_idx, int width);

    bool isLineFree(const ObstacleDistanceGrid& dist_grid,
                    float x0, float y0, float x1, float y1);

    int toIndex(int x, int y, int width) const { return y * width + x; }

    float inflation_radius_ = 0.2f;
};