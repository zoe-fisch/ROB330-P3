#pragma once

#include <mbot_interfaces/msg/pid_debug.hpp>

// Shared debug channel for all controllers.
//
// A controller is a plain class with no ROS node, so it cannot publish anything
// itself. Instead it records what it computed into these members, and
// ControllerNode reads them back each tick to publish /debug_cmd_vel and to
// append a row to the CSV log.
//
// Every controller inherits this so that switching `using Controller = ...`
// in controller_node.cpp keeps compiling. A controller that never calls
// recordDebug() simply leaves debug_valid_ false and produces no debug output.
class ControllerDebug
{
public:
  // PID error terms from the most recent tick (published on /debug_cmd_vel).
  const mbot_interfaces::msg::PIDDebug & getLastDebug() const {return last_debug_;}

  // Active goal during the most recent tick (logged to CSV).
  double getDebugGoalX() const {return dbg_goal_x_;}
  double getDebugGoalY() const {return dbg_goal_y_;}

  // True only when the last tick actually ran the controller. Guards against
  // republishing stale values at 20 Hz once the plan is finished.
  bool hasDebug() const {return debug_valid_;}

protected:
  // Call at the top of computeVelocityCommands(), before any early return.
  void clearDebug() {debug_valid_ = false;}

  // Call at the end of computeVelocityCommands(), once cmd is final.
  void recordDebug(double target_distance, double signed_distance_error,
                   double signed_angle_error, double target_angle,
                   double goal_x, double goal_y)
  {
    last_debug_.target_distance       = target_distance;
    last_debug_.signed_distance_error = signed_distance_error;
    last_debug_.signed_angle_error    = signed_angle_error;
    last_debug_.target_angle          = target_angle;
    dbg_goal_x_  = goal_x;
    dbg_goal_y_  = goal_y;
    debug_valid_ = true;
  }

private:
  mbot_interfaces::msg::PIDDebug last_debug_;
  double dbg_goal_x_{0.0}, dbg_goal_y_{0.0};
  bool debug_valid_{false};
};
