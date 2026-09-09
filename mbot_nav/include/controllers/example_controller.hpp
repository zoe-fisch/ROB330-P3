#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vector>
#include "utils.hpp"
#include "controllers/controller_debug.hpp"

// ExampleController — starter template for implementing a custom controller.
// TODO: Replace the placeholder bodies in example_controller.cpp with your algorithm.
class ExampleController : public ControllerDebug
{
public:
    ExampleController() = default;

    // Load the waypoints for this controller to follow.
    // current_x, current_y: the robot's position at the time the plan is received.
    void setPlan(const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
                 double current_x, double current_y);

    // Compute and return cmd_vel given the robot's current pose.
    // Returns zero twist when there is no active plan.
    geometry_msgs::msg::Twist computeVelocityCommands(double x, double y, double theta);

private:
    std::vector<geometry_msgs::msg::Pose2D> waypoints_;
    bool has_plan_{false};

    // TODO: Add your controller state variables here.
};
