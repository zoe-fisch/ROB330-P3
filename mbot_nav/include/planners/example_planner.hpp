#pragma once

#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include "obstacle_distance_grid.hpp"

// ExamplePlanner — starter template for implementing a custom path planner.
// TODO: Replace the placeholder body in example_planner.cpp with your algorithm.
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

    // TODO: Add private helper methods and member variables here.
};
