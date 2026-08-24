#pragma once

#include "common/action_model.hpp"
#include "common/sensor_model.hpp"
#include "common/mapping.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>

#include <vector>
#include <random>

// ParticleFilter class
class ParticleFilter
{
public:
    // Configuration constants
    static constexpr int NUM_PARTICLES = 400;

    ParticleFilter();

    // Initialize all particles at 'pose' and set the action model odometry baseline.
    void initialize(const geometry_msgs::msg::Pose& pose, const nav_msgs::msg::Odometry& odom);

    // Main update cycle
    // Full MCL step: prediction + measurement + resample
    // Returns best-estimate pose in the map frame
    geometry_msgs::msg::Pose update(const nav_msgs::msg::Odometry&      odom,
                                    const sensor_msgs::msg::LaserScan&  scan,
                                    const nav_msgs::msg::OccupancyGrid& map);

    // Getters
    const geometry_msgs::msg::Pose&       poseEstimate() const { return pose_estimate_; }
    const nav2_msgs::msg::ParticleCloud&  particleCloud() const { return particle_cloud_; }
    SensorModel&                          sensorModel()        { return sensor_model_; }

private:
    nav2_msgs::msg::ParticleCloud resampleParticles(const nav2_msgs::msg::ParticleCloud& cloud) const;
    geometry_msgs::msg::Pose computeEstimatePose(const nav2_msgs::msg::ParticleCloud& cloud) const;

    // (Optional) Replace a fraction of low-quality particles with random poses
    void reinvigorateParticles(nav2_msgs::msg::ParticleCloud& cloud);

    // Data members
    int                           num_particles_;   // number of particles
    nav2_msgs::msg::ParticleCloud particle_cloud_;  // particle cloud with header (frame_id, timestamp) and particles array
    geometry_msgs::msg::Pose      pose_estimate_;   // single best guess

    ActionModel               action_model_;    // motion model
    SensorModel               sensor_model_;    // laser likelihood

    mutable std::mt19937      random_gen;       // pseudo-random generator for resampling
};