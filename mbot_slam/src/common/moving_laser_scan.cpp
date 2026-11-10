#include "common/moving_laser_scan.hpp"
#include "common/utils.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Linear interpolation for geometry_msgs::msg::Point
inline geometry_msgs::msg::Point linearInterpolatePoint(const geometry_msgs::msg::Point& a,
                                           const geometry_msgs::msg::Point& b,
                                           double t)
{
    geometry_msgs::msg::Point out;
    out.x = a.x + t * (b.x - a.x);
    out.y = a.y + t * (b.y - a.y);
    out.z = a.z + t * (b.z - a.z);
    return out;
}


namespace {
MovingLaserScan::Pose toPose(const tf2::Transform& transform)
{
    MovingLaserScan::Pose pose;
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation = tf2::toMsg(transform.getRotation());
    return pose;
}
}

MovingLaserScan::MovingLaserScan(const sensor_msgs::msg::LaserScan& scan,
                                 const RelativePoses& relative_poses,
                                 const Pose& base_end_pose)
{
    tf2::Transform world_from_base;
    tf2::fromMsg(base_end_pose, world_from_base);
    deskewScan(scan, toPose(world_from_base * relative_poses.start),
                     toPose(world_from_base * relative_poses.end));
}

MovingLaserScan::MovingLaserScan(const sensor_msgs::msg::LaserScan& scan,
                                 const Pose& start_pose,
                                 const Pose& end_pose)
{
    deskewScan(scan, start_pose, end_pose);
}

void MovingLaserScan::deskewScan(const sensor_msgs::msg::LaserScan& scan,
                                     const Pose& start_pose,
                                     const Pose& end_pose)
{
    const size_t num_rays = scan.ranges.size();
    rays_.clear();
    rays_.reserve(num_rays);

    if (!std::isfinite(scan.range_max) || scan.range_max <= 0.0f ||
        !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment)) {
        return;
    }

    const double yaw_start = yawFromQuaternion(start_pose.orientation);
    const double delta_yaw = angleDiff(yawFromQuaternion(end_pose.orientation), yaw_start);

    for (size_t i = 0; i < num_rays; ++i) {
        const float range = scan.ranges[i];
        // Missing returns do not establish that the full sensor range was clear.
        if (!std::isfinite(range) || range <= 0.0f ||
            range < scan.range_min || range >= scan.range_max) {
            continue;
        }

        const double fraction = num_rays > 1 ? static_cast<double>(i) / (num_rays - 1) : 0.0;
        const double beam_angle = scan.angle_min + i * static_cast<double>(scan.angle_increment);

        InterpolatedRay ray;
        ray.origin = linearInterpolatePoint(start_pose.position, end_pose.position, fraction);
        ray.theta = wrapToPi(yaw_start + fraction * delta_yaw + beam_angle);
        ray.range = range;
        rays_.push_back(ray);
    }
}
