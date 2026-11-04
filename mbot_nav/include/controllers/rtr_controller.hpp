#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vector>
#include "utils.hpp"  // wrapToPi
#include "controllers/controller_debug.hpp"

// RTR (Rotate-Translate-Rotate) PID controller.
// Follows a queue of 2D waypoints. Call computeVelocityCommands() at 20 Hz (every 50 ms).
class RTRController : public ControllerDebug
{
public:
    RTRController() = default;

    // Load new waypoints; skips any already within reach of (current_x, current_y)
    void setPlan(const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
                 double current_x, double current_y);

    // Compute and return cmd_vel for the current goal given the robot's current pose.
    // Returns zero twist when there is no active plan.
    geometry_msgs::msg::Twist computeVelocityCommands(double x, double y, double theta);

private:
    double goal_x_{0}, goal_y_{0}, goal_theta_{0};
    bool   goal_received_{false};
    std::vector<geometry_msgs::msg::Pose2D> goal_queue_;

    static constexpr double dist_thresh_     = 0.05;
    static constexpr double angle_thresh_    = 0.05;
    static constexpr double dt_              = 0.05;   // 50 ms control period
    static constexpr double integral_limit_  = 1.0;

    // Used by applyRotationDeadband() (rad/s)
    static constexpr double min_rotation_speed_ = 0.2;
    static constexpr double max_angular_speed_  = 1.0;

    // PID gains
    static constexpr double Kp_lin = 1.0, Ki_lin = 0.0, Kd_lin = 0.01;
    static constexpr double Kp_ang = 1.0, Ki_ang = 0.0, Kd_ang = 0.05;

    // PID state
    double lin_error_sum_{0}, lin_error_last_{0};
    double ang_error_sum_{0}, ang_error_last_{0};
    
    double applyRotationDeadband(double angular_velocity) const;
    void resetPID();
    void loadNextGoal();
};
