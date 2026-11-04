#include "controllers/rtr_controller.hpp"
#include <algorithm>  // std::clamp
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// ============================================================
// Load waypoint plan
// ============================================================

void RTRController::setPlan(
    const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
    double current_x,
    double current_y)
{
    if (waypoints.empty())
    {
        goal_received_ = false;
        goal_queue_.clear();
        return;
    }
    // --------------------------------------------------------
    // Clear old plan and load new one
    // --------------------------------------------------------

    goal_queue_.clear();
    for (const auto& waypoint : waypoints)
    {
        goal_queue_.push_back(waypoint);
    }
    // --------------------------------------------------------
    // Skip waypoints that are already reached
    // --------------------------------------------------------

    while (!goal_queue_.empty())
    {
        const auto& waypoint = goal_queue_.front();
        const double dx = waypoint.x - current_x;
        const double dy = waypoint.y - current_y;
        const double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < dist_thresh_)
        {
            goal_queue_.erase(goal_queue_.begin());
        }
        else
        {
            break;
        }
    }
    // --------------------------------------------------------
    // Load first active goal
    // --------------------------------------------------------
    if (!goal_queue_.empty())
    {
        goal_x_ = goal_queue_.front().x;
        goal_y_ = goal_queue_.front().y;
        goal_theta_ = goal_queue_.front().theta;
        goal_queue_.erase(goal_queue_.begin());
        goal_received_ = true;

        resetPID();

        RCLCPP_INFO(
            rclcpp::get_logger("rtr_controller"),
            "Starting goal: x=%.2f y=%.2f theta=%.2f",
            goal_x_,
            goal_y_,
            goal_theta_);
    }
    else
    {
        goal_received_ = false;
    }
}


// ============================================================
// Compute velocity command
// ============================================================

geometry_msgs::msg::Twist
RTRController::computeVelocityCommands(double x, double y, double theta)
{
    geometry_msgs::msg::Twist cmd;
    // --------------------------------------------------------
    // No active trajectory
    // --------------------------------------------------------
    if (!goal_received_)
    {
        return cmd;
    }

    // --------------------------------------------------------
    // Position error to current waypoint
    // --------------------------------------------------------
    const double dx = goal_x_ - x;
    const double dy = goal_y_ - y;
    const double dist_error = std::sqrt(dx * dx + dy * dy);

    // Heading required to face waypoint
    const double target_angle = std::atan2(dy, dx);
    double angle_error = wrapToPi(target_angle - theta);

    // ========================================================
    // STATE 1:
    // ROTATE toward waypoint
    // ========================================================
    // TODO - STATE 1: rotate toward target
         // There are parameters defined in rtr_controller.hpp
         // You don't have to use all of them, treat them as hints
         // hint: lin_error_sum_ relates to p gain
         //       lin_error_last_ relates to i gain
         //       for d gain, you need to calculate the derivative. dt_ also defined in the header file
    // Hint: if dist_error is greater than dist_thresh_ AND if the angle_error is larger than a threshold, apply angular PID, return cmd;

    // if ((dist_error > dist_thresh_) && (std::fabs(angle_error) > angle_thresh_))
    // {
    
 

    // return cmd;
    // }



    // ========================================================
    // STATE 2:
    // TRANSLATE toward waypoint
    // ========================================================
    // TODO - STATE 2: drive forward with heading correction
    // Hint: if dist_error is greater than dist_thresh_, apply linear PID as well as heading correction (also an angular PID). return cmd;

    // if (dist_error > dist_thresh_)
    // {
    // // ----------------------------------------------------
    // // TODO 2.1  - Linear PID
    // // ----------------------------------------------------
    
    // // ----------------------------------------------------
    // // TODO 2.2 - Heading correction while translating
    // // ----------------------------------------------------
    
    //     return cmd;
    // }


    // ========================================================
    // STATE 3:
    // Position reached.
    // Rotate to waypoint's requested final orientation.
    // ========================================================
    // TODO - STATE 3: rotate to final orientation
    // Hint: 1. compute angle_error between goal and current theta; 
    // 2.  if the angle_error is larger than angle_thresh_, do an angular PID, return cmd. You can also choose to applyRotationDeadband to a final in-place rotation.
    
        
     // angle_error = wrapToPi(goal_theta_ - theta);
    // if (std::fabs(angle_error) > angle_thresh_)
     // {
  
        // // ----------------------------------------------------
        // // TODO 3.1  - final rotate in place angular PID
       // // ----------------------------------------------------




    //     return cmd;
    // }



    // ========================================================
    // WAYPOINT COMPLETE
    // ========================================================
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    RCLCPP_INFO(
        rclcpp::get_logger("rtr_controller"),
        "Reached waypoint: x=%.2f y=%.2f theta=%.2f",
        goal_x_,
        goal_y_,
        goal_theta_);

    // // Safety limits 
    // cmd.linear.x  = std::clamp(cmd.linear.x,  -0.3, 0.3);
    // cmd.angular.z = std::clamp(cmd.angular.z, -1.0, 1.0);

    // // Hand this tick's errors to ControllerNode for /debug_cmd_vel and the CSV log.
    // // signed_distance_error is the goal offset projected onto the robot heading.
    // double signed_distance_error = dx * std::cos(theta) + dy * std::sin(theta);
    // recordDebug(dist_error, signed_distance_error, angle_error, target_angle,
    //             goal_x_, goal_y_);

    resetPID();
    loadNextGoal();

    return cmd;
}


// ============================================================
// Apply minimum physical rotation speed - OPTIONAL, debug tool
// ============================================================

double RTRController::applyRotationDeadband(
    double angular_velocity) const
{
    // No rotation requested
    if (std::fabs(angular_velocity) < 1e-6)
    {
        return 0.0;
    }

    // If controller requests a very small rotation command,
    // raise it enough to overcome physical motor friction.
    if (std::fabs(angular_velocity) <
        min_rotation_speed_)
    {
        angular_velocity = std::copysign( min_rotation_speed_, angular_velocity);
    }

    return std::clamp( angular_velocity,  -max_angular_speed_,  max_angular_speed_);
}
// ============================================================
// Reset PID state
// ============================================================
void RTRController::resetPID()
{
    lin_error_sum_ = lin_error_last_ = 0.0;
    ang_error_sum_ = ang_error_last_ = 0.0;
}

// ============================================================
// Load next waypoint
// ============================================================
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
