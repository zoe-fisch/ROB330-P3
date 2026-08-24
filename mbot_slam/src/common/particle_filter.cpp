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
                                                const nav_msgs::msg::OccupancyGrid& map)
{
    bool moved = action_model_.processOdometry(odom);
    if (moved) {
        // only update estimate pose if the robot has moved
        const nav2_msgs::msg::ParticleCloud parent_cloud = particle_cloud_;
        nav2_msgs::msg::ParticleCloud proposal  = action_model_.propagateParticles(particle_cloud_);
        nav2_msgs::msg::ParticleCloud posterior = sensor_model_.weightParticles(proposal, parent_cloud, scan, map);
        pose_estimate_  = computeEstimatePose(posterior);
        particle_cloud_ = resampleParticles(posterior);
    }

    return pose_estimate_;
}

nav2_msgs::msg::ParticleCloud ParticleFilter::resampleParticles(const nav2_msgs::msg::ParticleCloud& cloud) const
{
    nav2_msgs::msg::ParticleCloud resampled;
    resampled.particles.reserve(num_particles_);

    std::uniform_real_distribution<double> unif(0.0, 1.0 / num_particles_);
    double random_offset = unif(random_gen);

    // TODO: resample the particles here
    (void)random_offset; (void)cloud;

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
    double x=0.0, y=0.0, cos_sum=0.0, sin_sum=0.0;

    for (const auto& p : cloud.particles) {
        const double w = p.weight;
        const double theta = yawFromQuaternion(p.pose.orientation);

        // TODO: calculate the weighted sum of the particles' pose
        (void)x; (void)y; (void)cos_sum; (void)sin_sum; (void)w, (void)theta;
    }

    geometry_msgs::msg::Pose est;
    // est.position.x = ?
    // est.position.y = ?
    // est.orientation = quaternionFromYaw(?);
    return est;
}