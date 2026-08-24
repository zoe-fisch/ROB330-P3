#include "controllers/pure_pursuit_controller.hpp"
#include <algorithm>
#include <cmath>

void PurePursuitController::setPlan(
    const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
    double /*current_x*/, double /*current_y*/)
{
    path_     = waypoints;
}

geometry_msgs::msg::Twist PurePursuitController::computeVelocityCommands(double x, double y, double theta)
{
    geometry_msgs::msg::Twist cmd;
    if (path_.empty()) return cmd;
    // TODO: implement pure pursuit logic here
    // There are parameters defined in pure_pursuit_controller.hpp
    // You don't have to use all of them, treat them as hints
    (void)x; (void)y; (void)theta;

    return cmd;
}

bool PurePursuitController::findLookaheadPoint(
    double x, double y, double& target_x, double& target_y)
{
    // TODO: implement lookahead on the path logic
    (void)x; (void)y; (void)target_x; (void)target_y;
    return false;
}
