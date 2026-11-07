#include "common/moving_laser_scan.hpp"
#include "common/utils.hpp"
#include <cmath>

// Linear interpolation for geometry_msgs::msg::Point
inline geometry_msgs::msg::Point linearInterpolatePoint(const geometry_msgs::msg::Point& a,
                                           const geometry_msgs::msg::Point& b,
                                           float t)
{
    geometry_msgs::msg::Point out;
    out.x = a.x + t * (b.x - a.x);
    out.y = a.y + t * (b.y - a.y);
    out.z = a.z + t * (b.z - a.z);
    return out;
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

    // Extract yaw from quaternions (world-frame base yaw at scan start/end)
    const double yaw_start = yawFromQuaternion(start_pose.orientation);
    const double yaw_end = yawFromQuaternion(end_pose.orientation);
    
    // TODO: Calculate how much the robot body has rotated in total - delta_yaw
    double delta_yaw = wrapToPi(yaw_end - yaw_start);
    
    // Iterate over all beams
    for (size_t i = 0; i < num_rays; ++i)
    {
        const float range = scan.ranges[i];

        // Drop garbage / under-range / out-of-range
        if (std::isnan(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }

        // TODO: Calculate how much this ray has rotated - delta_lidar
        // Hint: To calculate delta_lidar, check LiDAR Scan’s ROS message type “sensor_msgs/LaserScan” 

        double delta_lidar = i * scan.angle_increment + scan.angle_min;
    
        // TODO: ray.theta = delta_lidar + a fraction of delta_yaw
        double fraction_of_delta_yaw;
        if (num_rays > 1){
            fraction_of_delta_yaw = float(i) / float(num_rays - 1);
        }
        else {
            fraction_of_delta_yaw = 0;
        }


        InterpolatedRay ray;
        ray.origin = linearInterpolatePoint(start_pose.position, end_pose.position, fraction_of_delta_yaw);
        ray.theta  = wrapToPi(yaw_start + fraction_of_delta_yaw * delta_yaw + delta_lidar);
        ray.range  = range;

        rays_.push_back(ray);
    }
}