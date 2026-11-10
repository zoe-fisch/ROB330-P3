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

    if (delta_trans < min_trans_ && std::abs(delta_theta) < min_rot_) {
        return false;
    }

    rot1_ = delta_trans > 1e-6 ? angleDiff(std::atan2(dy, dx), theta_prev) : 0.0;
    trans_ = delta_trans;
    // Represent reverse motion as negative translation to avoid two spurious half-turns.
    if (std::abs(rot1_) > M_PI_2) {
        rot1_ = angleDiff(rot1_, M_PI);
        trans_ = -trans_;
    }
    rot2_ = angleDiff(delta_theta, rot1_);

    rot1_std_ = std::sqrt(k1_ * std::abs(rot1_));
    trans_std_ = std::sqrt(k2_ * std::abs(trans_));
    rot2_std_ = std::sqrt(k1_ * std::abs(rot2_));

    // Retain sub-threshold motion for the next update.
    prev_odom_ = odom;
    return true;
}

geometry_msgs::msg::Pose ActionModel::propagateParticle(const geometry_msgs::msg::Pose& pose)
{
    const auto sample = [this](double mean, double stddev) {
        return stddev > 0.0 ? std::normal_distribution<double>(mean, stddev)(random_gen) : mean;
    };
    const double sampled_rot1 = sample(rot1_, rot1_std_);
    const double sampled_trans = sample(trans_, trans_std_);
    const double sampled_rot2 = sample(rot2_, rot2_std_);

    const double heading = yawFromQuaternion(pose.orientation) + sampled_rot1;
    geometry_msgs::msg::Pose new_pose = pose;
    new_pose.position.x += sampled_trans * std::cos(heading);
    new_pose.position.y += sampled_trans * std::sin(heading);
    new_pose.orientation = quaternionFromYaw(heading + sampled_rot2);

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
