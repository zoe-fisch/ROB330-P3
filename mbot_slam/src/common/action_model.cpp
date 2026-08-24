#include "common/action_model.hpp"
#include "common/utils.hpp"
#include <nav2_msgs/msg/particle.hpp>

#include <random>
#include <cmath>
#include <rclcpp/rclcpp.hpp>


ActionModel::ActionModel()
{
    std::random_device rd;
    random_gen = std::mt19937(rd());
}

void ActionModel::initialize(const nav_msgs::msg::Odometry& odom)
{
    prev_odom_   = odom;
    RCLCPP_INFO(rclcpp::get_logger("action_model"), "Initialized action model odometry reference.");
}

bool ActionModel::processOdometry(const nav_msgs::msg::Odometry& odom)
{
    // Delta translation in the odom frame
    const double dx = odom.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    const double dy = odom.pose.pose.position.y - prev_odom_.pose.pose.position.y;

    // Extract planar headings
    const double theta_prev = yawFromQuaternion(prev_odom_.pose.pose.orientation);
    const double theta_curr = yawFromQuaternion(odom.pose.pose.orientation);

    const double delta_trans  = std::sqrt(dx*dx + dy*dy);
    const double delta_theta  = angleDiff(theta_curr, theta_prev);

    rot1_ = angleDiff(std::atan2(dy, dx), theta_prev);
    trans_ = delta_trans;
    rot2_ = angleDiff(delta_theta, rot1_);

    const bool moved = (delta_trans >= min_trans_) ||
        (std::abs(delta_theta) >= min_rot_);

    if (moved) {
        (void)rot1_; (void)rot2_; (void)trans_;
        // TODO: Compute the standard deviation (sqrt of variance) for each motion component.
        // Noise scales with motion: std = sqrt(k1 * motion)
        // k1, k2 are defined in action_model.hpp.
        // rot1_std_  = ?
        // trans_std_ = ?
        // rot2_std_  = ?
    }

    prev_odom_ = odom;

    return moved;
}

geometry_msgs::msg::Pose ActionModel::propagateParticle(const geometry_msgs::msg::Pose& pose)
{
    // Sample noisy motion components
    std::normal_distribution<double> rot1_distribution(rot1_,  rot1_std_);
    std::normal_distribution<double> trans_distribution(trans_, trans_std_);
    std::normal_distribution<double> rot2_distribution (rot2_,  rot2_std_);

    const double sampled_rot1   = rot1_distribution(random_gen);
    const double sampled_trans  = trans_distribution(random_gen);
    const double sampled_rot2   = rot2_distribution(random_gen);

    // Current heading of the particle
    const double theta = yawFromQuaternion(pose.orientation);

    // TODO: with sampled R-T-R values, what is the new pose of the particle
    geometry_msgs::msg::Pose new_pose = pose;
    (void)sampled_rot1; (void)sampled_rot2; (void)sampled_trans; (void)theta;
    // new_pose.position.x = ?
    // new_pose.position.y = ?
    // new_pose.orientation = quaternionFromYaw(?);

    return new_pose;
}

nav2_msgs::msg::ParticleCloud ActionModel::propagateParticles(const nav2_msgs::msg::ParticleCloud& cloud)
{
    nav2_msgs::msg::ParticleCloud proposal;
    proposal.particles.reserve(cloud.particles.size());

    for (const auto& particle : cloud.particles) {
        nav2_msgs::msg::Particle q = particle;
        q.pose = propagateParticle(particle.pose);
        proposal.particles.push_back(std::move(q));
    }
    return proposal;
}
