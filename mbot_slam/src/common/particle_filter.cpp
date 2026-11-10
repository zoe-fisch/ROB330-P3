#include "common/particle_filter.hpp"
#include "common/utils.hpp"
#include "common/moving_laser_scan.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

// Constructor
ParticleFilter::ParticleFilter()
: num_particles_(NUM_PARTICLES),
  action_model_(),
  sensor_model_()
{
    particle_cloud_.particles.resize(num_particles_);

    std::random_device seed_source;
    random_gen = std::mt19937(seed_source());
}

void ParticleFilter::initialize(const geometry_msgs::msg::Pose& pose,
                                const nav_msgs::msg::Odometry& odom)
{
    const double w = 1.0 / static_cast<double>(num_particles_);
    RCLCPP_INFO(rclcpp::get_logger("particle_filter"), 
                "uniform weight initialization %f", w);
    for (auto& p : particle_cloud_.particles) {
        p.pose   = pose;
        p.weight = w;
    }
    pose_estimate_ = pose;
    action_model_.initialize(odom);
}

// Main update cycle
geometry_msgs::msg::Pose ParticleFilter::update(const nav_msgs::msg::Odometry&     odom,
                                                const sensor_msgs::msg::LaserScan& scan,
                                                const MovingLaserScan::RelativePoses& scan_poses,
                                                const nav_msgs::msg::OccupancyGrid& map)
{
    bool moved = action_model_.processOdometry(odom);
    if (moved) {
        // only update estimate pose if the robot has moved
        nav2_msgs::msg::ParticleCloud proposal  = action_model_.propagateParticles(particle_cloud_);
        nav2_msgs::msg::ParticleCloud posterior = sensor_model_.weightParticles(proposal, scan, scan_poses, map);
        pose_estimate_  = computeEstimatePose(posterior);
        particle_cloud_ = resampleParticles(posterior);
    }

    return pose_estimate_;
}

nav2_msgs::msg::ParticleCloud ParticleFilter::resampleParticles(const nav2_msgs::msg::ParticleCloud& cloud) const
{
    nav2_msgs::msg::ParticleCloud resampled;
    resampled.particles.reserve(num_particles_);

    if (cloud.particles.empty()) return resampled;

    const double step = 1.0 / num_particles_;
    const double offset = std::uniform_real_distribution<double>(0.0, step)(random_gen);
    size_t index = 0;
    double cumulative_weight = cloud.particles[index].weight;
    for (int i = 0; i < num_particles_; ++i) {
        const double target = offset + i * step;
        while (target >= cumulative_weight && index + 1 < cloud.particles.size()) {
            cumulative_weight += cloud.particles[++index].weight;
        }
        resampled.particles.push_back(cloud.particles[index]);
        resampled.particles.back().weight = step;
    }

    return resampled;
}

// (Optional) Replace a fraction of low-quality particles with random poses
// to recover from catastrophic localization failure ("kidnapped robot").
void ParticleFilter::reinvigorateParticles(nav2_msgs::msg::ParticleCloud& cloud)
{
    (void)cloud;
}

geometry_msgs::msg::Pose ParticleFilter::computeEstimatePose(const nav2_msgs::msg::ParticleCloud& cloud) const
{
    geometry_msgs::msg::Pose est;
    double weight_sum = 0.0;
    double cos_sum = 0.0;
    double sin_sum = 0.0;

    for (const auto& p : cloud.particles) {
        const double w = p.weight;
        const double theta = yawFromQuaternion(p.pose.orientation);
        est.position.x += w * p.pose.position.x;
        est.position.y += w * p.pose.position.y;
        est.position.z += w * p.pose.position.z;
        cos_sum += w * std::cos(theta);
        sin_sum += w * std::sin(theta);
        weight_sum += w;
    }

    if (weight_sum <= 0.0) return pose_estimate_;
    est.position.x /= weight_sum;
    est.position.y /= weight_sum;
    est.position.z /= weight_sum;
    est.orientation = std::hypot(cos_sum, sin_sum) > 1e-9
        ? quaternionFromYaw(std::atan2(sin_sum, cos_sum))
        : pose_estimate_.orientation;
    return est;
}
