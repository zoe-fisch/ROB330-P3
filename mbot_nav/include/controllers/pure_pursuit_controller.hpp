#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vector>
#include "utils.hpp"
#include "controllers/controller_debug.hpp"

// Pure Pursuit controller — geometric path tracker.
// Call computeVelocityCommands() at ~20 Hz. All waypoints are in the world frame.
class PurePursuitController : public ControllerDebug
{
public:
    PurePursuitController() = default;

    // Load the full path. Unlike RTR, Pure Pursuit needs the entire path at once.
    void setPlan(const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
                 double current_x, double current_y);

    // Compute and return cmd_vel toward the lookahead point on the path.
    // Returns zero twist when there is no active plan.
    geometry_msgs::msg::Twist computeVelocityCommands(double x, double y, double theta);

    // Tunable parameters
    double lookahead_distance  = 0.3;   // meters — how far ahead to steer toward
    double linear_speed        = 0.15;  // m/s — constant forward speed
    double angular_speed_limit = 1.0;   // rad/s — cap on angular velocity
    double goal_threshold      = 0.05;   // meters — position tolerance for final goal
    double heading_threshold   = 0.1;   // rad   — heading tolerance for final goal
    double angular_gain        = 2.0;   // P gain for final heading correction

private:
    std::vector<geometry_msgs::msg::Pose2D> path_;

    // Find and return the lookahead point; pops passed waypoints from path_.
    // Returns false only when one waypoint remains (handled by goal check).
    bool findLookaheadPoint(double x, double y, double& lx, double& ly);
};