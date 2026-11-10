#pragma once

#include "common/moving_laser_scan.hpp"

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

inline rclcpp::Time scanLastRayTime(const sensor_msgs::msg::LaserScan& scan)
{
    const rclcpp::Time start(scan.header.stamp);
    if (scan.ranges.size() < 2 || !std::isfinite(scan.time_increment) ||
        scan.time_increment <= 0.0f) {
        return start;
    }

    return start + rclcpp::Duration::from_seconds(
        static_cast<double>(scan.time_increment) * (scan.ranges.size() - 1));
}

inline MovingLaserScan::RelativePoses lookupScanRelativePoses(
    const tf2_ros::Buffer& buffer, const sensor_msgs::msg::LaserScan& scan,
    const geometry_msgs::msg::TransformStamped& odom_from_base_end)
{
    const auto timeout = rclcpp::Duration::from_seconds(0.1);
    tf2::Transform base_end_from_odom, odom_from_lidar_start, base_end_from_lidar_end;
    tf2::fromMsg(odom_from_base_end.transform, base_end_from_odom);
    base_end_from_odom = base_end_from_odom.inverse();
    tf2::fromMsg(buffer.lookupTransform("odom", scan.header.frame_id,
                                        rclcpp::Time(scan.header.stamp), timeout).transform,
                 odom_from_lidar_start);
    tf2::fromMsg(buffer.lookupTransform("base_footprint", scan.header.frame_id,
                                        rclcpp::Time(odom_from_base_end.header.stamp), timeout).transform,
                 base_end_from_lidar_end);
    return {base_end_from_odom * odom_from_lidar_start, base_end_from_lidar_end};
}
