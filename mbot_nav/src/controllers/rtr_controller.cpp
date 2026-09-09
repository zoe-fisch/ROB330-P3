#include "controllers/rtr_controller.hpp"
#include <algorithm>  // std::clamp

void RTRController::setPlan(const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
                            double current_x, double current_y)
{
    if (waypoints.empty()) return;

    goal_queue_.clear();
    for (const auto& p : waypoints) goal_queue_.push_back(p);

    // Skip waypoints already within reach
    while (!goal_queue_.empty()) {
        const auto& front = goal_queue_.front();
        double dx = front.x - current_x;
        double dy = front.y - current_y;
        if (std::sqrt(dx*dx + dy*dy) < dist_thresh_) {
            goal_queue_.erase(goal_queue_.begin());
        } else {
            break;
        }
    }

    if (!goal_queue_.empty()) {
        goal_x_       = goal_queue_.front().x;
        goal_y_       = goal_queue_.front().y;
        goal_theta_   = goal_queue_.front().theta;
        goal_queue_.erase(goal_queue_.begin());
        goal_received_ = true;
    } else {
        goal_received_ = false;
    }
}

geometry_msgs::msg::Twist RTRController::computeVelocityCommands(double x, double y, double theta)
{
    geometry_msgs::msg::Twist cmd;
    clearDebug();
    if (!goal_received_) return cmd;

    double dx           = goal_x_ - x;
    double dy           = goal_y_ - y;
    double target_angle = std::atan2(dy, dx);
    double dist_error     = std::sqrt(dx*dx + dy*dy);
    double angle_error  = wrapToPi(target_angle - theta);

    if (dist_error > dist_thresh_ && std::fabs(angle_error) > angle_thresh_) {
        // TODO - STATE 1: rotate toward target
        // There are parameters defined in rtr_controller.hpp
        // You don't have to use all of them, treat them as hints
        // hint: lin_error_sum_ relates to p gain
        //       lin_error_last_ relates to i gain
        //       for d gain, you need to calculate the derivative. dt_ also defined in the header file
    }
    else if (dist_error > dist_thresh_) {
        // TODO - STATE 2: drive forward with heading correction
    }
    else {
        // TODO - STATE 3: rotate to final orientation
    }

    // Safety limits
    cmd.linear.x  = std::clamp(cmd.linear.x,  -0.3, 0.3);
    cmd.angular.z = std::clamp(cmd.angular.z, -1.0, 1.0);

    // Hand this tick's errors to ControllerNode for /debug_cmd_vel and the CSV log.
    // signed_distance_error is the goal offset projected onto the robot heading.
    double signed_distance_error = dx * std::cos(theta) + dy * std::sin(theta);
    recordDebug(dist_error, signed_distance_error, angle_error, target_angle,
                goal_x_, goal_y_);

    return cmd;
}

void RTRController::resetPID()
{
    lin_error_sum_ = lin_error_last_ = 0.0;
    ang_error_sum_ = ang_error_last_ = 0.0;
}

void RTRController::loadNextGoal()
{
    if (!goal_queue_.empty()) {
        goal_x_     = goal_queue_.front().x;
        goal_y_     = goal_queue_.front().y;
        goal_theta_ = goal_queue_.front().theta;
        goal_queue_.erase(goal_queue_.begin());
    } else {
        goal_received_ = false;
    }
}
