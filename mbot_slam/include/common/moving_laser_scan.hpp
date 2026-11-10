#pragma once

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.h>

struct InterpolatedRay
{
    geometry_msgs::msg::Point origin;
    float theta;
    float range;
};

class MovingLaserScan
{
public:
    using Pose = geometry_msgs::msg::Pose;

    // Both transforms express the LiDAR pose relative to the robot base at the last ray.
    struct RelativePoses {
        tf2::Transform start;
        tf2::Transform end;
    };

    MovingLaserScan(const sensor_msgs::msg::LaserScan& scan,
                    const RelativePoses& relative_poses,
                    const Pose& base_end_pose);

    MovingLaserScan(const sensor_msgs::msg::LaserScan& scan,
                    const Pose& start_pose,
                    const Pose& end_pose);

    std::vector<InterpolatedRay>::const_iterator begin() const { return rays_.begin(); }
    std::vector<InterpolatedRay>::const_iterator end() const { return rays_.end(); }

private:
    std::vector<InterpolatedRay> rays_;

    void deskewScan(const sensor_msgs::msg::LaserScan& scan,
                        const Pose& start_pose,
                        const Pose& end_pose);
};
