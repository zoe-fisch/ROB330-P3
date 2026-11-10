#include "controllers/rtr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

void RTRController::setPlan(
    const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
    double current_x, double current_y)
{
    auto first = waypoints.begin();
    while (first != waypoints.end() && first + 1 != waypoints.end() &&
           std::hypot(first->x - current_x, first->y - current_y) <= dist_thresh_) {
        ++first;
    }
    goal_queue_.assign(first, waypoints.end());
    loadNextGoal();
}

geometry_msgs::msg::Twist
RTRController::computeVelocityCommands(double x, double y, double theta)
{
    clearDebug();
    geometry_msgs::msg::Twist cmd;
    if (!goal_received_) return cmd;

    const double dx = goal_x_ - x;
    const double dy = goal_y_ - y;
    const double dist_error = std::hypot(dx, dy);

    const bool final_alignment = dist_error <= dist_thresh_;
    if (final_alignment != final_alignment_) {
        resetPID();
        final_alignment_ = final_alignment;
    }
    const double target_angle = final_alignment ? goal_theta_ : std::atan2(dy, dx);
    const double angle_error = wrapToPi(target_angle - theta);
    const double signed_distance = dx * std::cos(theta) + dy * std::sin(theta);
    recordDebug(dist_error, signed_distance, angle_error, target_angle, goal_x_, goal_y_);

    if (final_alignment && std::abs(angle_error) <= angle_thresh_) {
        RCLCPP_INFO(rclcpp::get_logger("rtr_controller"),
                    "Reached waypoint: x=%.2f y=%.2f theta=%.2f", goal_x_, goal_y_, goal_theta_);
        loadNextGoal();
        return cmd;
    }

    const double angle_derivative = have_ang_error_
        ? wrapToPi(angle_error - ang_error_last_) / dt_ : 0.0;
    ang_error_sum_ = std::clamp(ang_error_sum_ + angle_error * dt_, -integral_limit_, integral_limit_);
    ang_error_last_ = angle_error;
    have_ang_error_ = true;
    const double turn = Kp_ang * angle_error + Ki_ang * ang_error_sum_ + Kd_ang * angle_derivative;

    if (final_alignment || std::abs(angle_error) > angle_thresh_) {
        // Keep the rotation direction consistent with the heading error near the target.
        cmd.angular.z = applyRotationDeadband(std::copysign(std::abs(turn), angle_error));
        lin_error_sum_ = 0.0;
        have_lin_error_ = false;
        return cmd;
    }

    const double distance_derivative = have_lin_error_ ? (dist_error - lin_error_last_) / dt_ : 0.0;
    lin_error_sum_ = std::clamp(lin_error_sum_ + dist_error * dt_, -integral_limit_, integral_limit_);
    lin_error_last_ = dist_error;
    have_lin_error_ = true;
    cmd.linear.x = std::clamp(Kp_lin * dist_error + Ki_lin * lin_error_sum_ +
                              Kd_lin * distance_derivative, 0.0, 0.3);
    cmd.angular.z = std::clamp(turn, -max_angular_speed_, max_angular_speed_);
    return cmd;
}

double RTRController::applyRotationDeadband(double angular_velocity) const
{
    return std::copysign(std::clamp(std::abs(angular_velocity),
                                  min_rotation_speed_, max_angular_speed_), angular_velocity);
}

void RTRController::resetPID()
{
    lin_error_sum_ = lin_error_last_ = 0.0;
    ang_error_sum_ = ang_error_last_ = 0.0;
    have_lin_error_ = have_ang_error_ = false;
}

void RTRController::loadNextGoal()
{
    resetPID();
    final_alignment_ = false;
    goal_received_ = !goal_queue_.empty();
    if (!goal_received_) return;

    goal_x_ = goal_queue_.front().x;
    goal_y_ = goal_queue_.front().y;
    goal_theta_ = goal_queue_.front().theta;
    goal_queue_.erase(goal_queue_.begin());
}
