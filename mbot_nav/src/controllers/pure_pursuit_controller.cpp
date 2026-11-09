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
    clearDebug();
    if (path_.empty()) return cmd;

    const auto& goal = path_.back();
    const double goal_distance = std::hypot(goal.x - x, goal.y - y);
    if (goal_distance <= goal_threshold)
    {
        const double heading_error = wrapToPi(goal.theta - theta);
        if (std::fabs(heading_error) <= heading_threshold)
            path_.clear();
        else
            cmd.angular.z = std::clamp(angular_gain * heading_error,
                                       -angular_speed_limit, angular_speed_limit);
        return cmd;
    }

    double target_x = goal.x;
    double target_y = goal.y;
    findLookaheadPoint(x, y, target_x, target_y);
    const double dx = target_x - x;
    const double dy = target_y - y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared <= 1e-12) return cmd;

    const double lateral_error = -std::sin(theta) * dx + std::cos(theta) * dy;
    cmd.linear.x = std::min(linear_speed, goal_distance);
    cmd.angular.z = std::clamp(2.0 * cmd.linear.x * lateral_error / distance_squared,
                               -angular_speed_limit, angular_speed_limit);

    return cmd;
}

bool PurePursuitController::findLookaheadPoint(
    double x, double y, double& target_x, double& target_y)
{
    if (path_.empty()) return false;

    // Start at the closest projection onto the remaining path.
    std::size_t closest_segment = 0;
    double closest_distance_squared = INFINITY;
    double start_x = path_.front().x;
    double start_y = path_.front().y;
    for (std::size_t i = 0; i + 1 < path_.size(); ++i)
    {
        const double dx = path_[i + 1].x - path_[i].x;
        const double dy = path_[i + 1].y - path_[i].y;
        const double length_squared = dx * dx + dy * dy;
        const double t = length_squared > 0.0
            ? std::clamp(((x - path_[i].x) * dx + (y - path_[i].y) * dy)
                         / length_squared, 0.0, 1.0) : 0.0;
        const double px = path_[i].x + t * dx;
        const double py = path_[i].y + t * dy;
        const double distance_squared = (px - x) * (px - x) + (py - y) * (py - y);
        if (distance_squared < closest_distance_squared)
        {
            closest_distance_squared = distance_squared;
            closest_segment = i;
            start_x = px;
            start_y = py;
        }
    }
    if (closest_segment > 0)
        path_.erase(path_.begin(), path_.begin() + closest_segment);

    double remaining = std::max(lookahead_distance, 0.0);
    for (std::size_t i = 1; i < path_.size(); ++i)
    {
        const double dx = path_[i].x - start_x;
        const double dy = path_[i].y - start_y;
        const double length = std::hypot(dx, dy);
        if (length > 0.0 && remaining <= length)
        {
            target_x = start_x + remaining / length * dx;
            target_y = start_y + remaining / length * dy;
            return true;
        }
        remaining -= length;
        start_x = path_[i].x;
        start_y = path_[i].y;
    }
    target_x = path_.back().x;
    target_y = path_.back().y;
    return path_.size() > 1;
}
