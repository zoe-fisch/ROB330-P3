#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

// Wrap angle to (-π, π]
inline double wrapToPi(double angle)
{
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    while (angle >   M_PI) angle -= 2.0 * M_PI;
    return angle;
}

// Signed shortest difference a − b, wrapped to (-π, π]
inline double angleDiff(double a, double b)
{
    return wrapToPi(a - b);
}

// Extract planar yaw from quaternion
inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion q_tf2(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf2).getRPY(roll, pitch, yaw);
    return yaw;
}

// Build a quaternion from a planar yaw angle
inline geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}
