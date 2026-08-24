#include "controllers/example_controller.hpp"

void ExampleController::setPlan(
    const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
    double /*current_x*/, double /*current_y*/)
{
    // TODO: Store waypoints and initialize your controller state.
    waypoints_ = waypoints;
    has_plan_  = !waypoints_.empty();
}

geometry_msgs::msg::Twist ExampleController::computeVelocityCommands(
    double /*x*/, double /*y*/, double /*theta*/)
{
    geometry_msgs::msg::Twist cmd;
    if (!has_plan_) return cmd;

    // TODO: Implement your control law here.
    //       Set cmd.linear.x (forward speed, m/s) and
    //       cmd.angular.z (turning rate, rad/s), then return cmd.

    return cmd;
}
