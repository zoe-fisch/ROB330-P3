#include "planners/example_planner.hpp"

bool ExamplePlanner::planPath(
    const ObstacleDistanceGrid& dist_grid,
    const geometry_msgs::msg::Pose2D& start,
    const geometry_msgs::msg::Pose2D& goal,
    mbot_interfaces::msg::Pose2DArray& path)
{
    return planner_.planPath(dist_grid, start, goal, path);
}
