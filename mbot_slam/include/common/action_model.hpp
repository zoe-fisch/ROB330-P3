#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <random>
#include <cmath>

class ActionModel
{
public:
    ActionModel();

    // Set the initial odometry baseline so the action model can compute deltas.
    void initialize(const nav_msgs::msg::Odometry& odom);

    // Process a new odometry measurement.
    bool processOdometry(const nav_msgs::msg::Odometry& odom);

    // Propagate a single particle by applying noisy motion from the motion model.
    geometry_msgs::msg::Pose propagateParticle(const geometry_msgs::msg::Pose& pose);

    // Propagate all particles in a cloud and return the resulting proposal cloud.
    nav2_msgs::msg::ParticleCloud propagateParticles(const nav2_msgs::msg::ParticleCloud& cloud);

private:
    nav_msgs::msg::Odometry prev_odom_;

    double k1_{0.015};          // rotational noise coefficient
    double k2_{0.008};          // translational noise coefficient
    double min_trans_{0.005}; // [m] ignore motion below this
    double min_rot_ {0.01};    // [rad] ignore rotation below this

    // (rot1, trans, rot2) motion decomposition
    double rot1_{0.0}, trans_{0.0}, rot2_{0.0};
    /* Corresponding 1-σ values */
    double rot1_std_{0.0}, trans_std_{0.0}, rot2_std_{0.0};

    /* Random-number generator */
    std::mt19937 random_gen;
};