#include "planners/example_planner.hpp"

bool ExamplePlanner::planPath(
    const ObstacleDistanceGrid& /*dist_grid*/,
    const geometry_msgs::msg::Pose2D& /*start*/,
    const geometry_msgs::msg::Pose2D& /*goal*/,
    mbot_interfaces::msg::Pose2DArray& path)
{
    // TODO: Implement your path planning algorithm here.
    //       Populate path.poses with the planned waypoints and return true.
    //       Return false if no path can be found.
    path.poses.clear();
    return false;
}
