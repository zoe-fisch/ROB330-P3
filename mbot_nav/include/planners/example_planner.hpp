#pragma once

#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include "planners/astar_planner.hpp"

// Example planner that uses the obstacle-aware A* implementation.
class ExamplePlanner
{
public:
    ExamplePlanner() = default;

    // Plan a path from start to goal on the given obstacle distance grid.
    // Returns true and fills path.poses on success; returns false if no path is found.
    bool planPath(const ObstacleDistanceGrid& dist_grid,
                  const geometry_msgs::msg::Pose2D& start,
                  const geometry_msgs::msg::Pose2D& goal,
                  mbot_interfaces::msg::Pose2DArray& path);

private:
    AStarPlanner planner_;
};
