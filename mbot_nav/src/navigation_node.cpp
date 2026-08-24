#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <optional>
#include <cmath>
#include "obstacle_distance_grid.hpp"
#include "utils.hpp"

// To switch planners, change the include and the type of Planner below.
#include "planners/astar_planner.hpp"
// #include "planners/theta_star_planner.hpp"
// #include "planners/example_planner.hpp"

using Planner = AStarPlanner;
// using Planner = ThetaStarPlanner;
// using Planner = ExamplePlanner;

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode()
        : Node("navigation_node")
    {
        // Declare and get pose_source parameter
        this->declare_parameter<std::string>("pose_source", "manual");
        pose_source_ = this->get_parameter("pose_source").as_string();

        RCLCPP_INFO(this->get_logger(), "Pose source: %s", pose_source_.c_str());

        // Publishers
        waypoints_pub_ = this->create_publisher<mbot_interfaces::msg::Pose2DArray>(
            "/waypoints", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planned_path", 10);

        // Setup TF listener if using TF mode
        if (pose_source_ == "tf") {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Create a timer to periodically update robot pose from TF
            tf_update_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&NavigationNode::updateRobotPose, this));

            RCLCPP_INFO(this->get_logger(), "Using TF-based pose lookup");
        } else {
            // Subscribe to /initialpose topic in manual mode
            initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/initialpose",
                rclcpp::QoS(10),
                std::bind(&NavigationNode::initialPoseCallback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Using manual pose from RViz");
        }

        // Subscriptions
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&NavigationNode::mapCallback, this, std::placeholders::_1));

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            rclcpp::QoS(10),
            std::bind(&NavigationNode::goalPoseCallback, this, std::placeholders::_1));
    }

private:
    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    // Publishers
    rclcpp::Publisher<mbot_interfaces::msg::Pose2DArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Data
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::optional<geometry_msgs::msg::Pose2D> robot_pose_;
    std::optional<geometry_msgs::msg::Pose2D> goal_pose_;
    double goal_reached_threshold_ = 0.15;

    // Planners
    Planner planner_;
    ObstacleDistanceGrid dist_grid_;

    // TF support
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Configuration
    std::string pose_source_;  // "manual" or "tf"
    rclcpp::TimerBase::SharedPtr tf_update_timer_;

    // Subscribers callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = msg;
        dist_grid_.computeDistFromMap(*msg);
        // RCLCPP_INFO(this->get_logger(), "Updated distance grid from map");
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // This callback is used for visualizing planned path, for testing only
        geometry_msgs::msg::Pose2D pose2d;
        pose2d.x     = msg->pose.pose.position.x;
        pose2d.y     = msg->pose.pose.position.y;
        pose2d.theta = yawFromQuaternion(msg->pose.pose.orientation);

        robot_pose_ = pose2d;
        RCLCPP_INFO(this->get_logger(), "Received initial pose: (%.2f, %.2f, %.2f)",
                    pose2d.x, pose2d.y, pose2d.theta);

        // If we already have a goal, replan
        if (goal_pose_) {
            planPath();
        }
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Extract Pose2D from PoseStamped
        geometry_msgs::msg::Pose2D pose2d;
        pose2d.x     = msg->pose.position.x;
        pose2d.y     = msg->pose.position.y;
        pose2d.theta = yawFromQuaternion(msg->pose.orientation);

        goal_pose_ = pose2d;
        RCLCPP_INFO(this->get_logger(), "Received goal pose: (%.2f, %.2f, %.2f)",
                    pose2d.x, pose2d.y, pose2d.theta);

        // Plan path immediately if we have robot pose
        if (robot_pose_) {
            planPath();
        }
    }

    void updateRobotPose()
    {
        geometry_msgs::msg::Pose2D pose;
        try {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);

            // Extract position
            pose.x     = transform.transform.translation.x;
            pose.y     = transform.transform.translation.y;
            pose.theta = yawFromQuaternion(transform.transform.rotation);
            robot_pose_ = pose;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Robot pose TF map->base_footprint look up error: %s", ex.what());
            return;
        }

        // Check if goal has been reached
        if (goal_pose_) {
            double dx = pose.x - goal_pose_->x;
            double dy = pose.y - goal_pose_->y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= goal_reached_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                goal_pose_.reset();  // Clear goal so we stop replanning
            } else {
                // Still navigating to goal - replan with current robot pose
                planPath();
            }
        }
    }

    // Planning
    void planPath()
    {
        if (!latest_map_) {
            RCLCPP_WARN(this->get_logger(), "No map available yet");
            return;
        }

        if (!robot_pose_ || !goal_pose_) {
            RCLCPP_WARN(this->get_logger(), "Missing robot_pose or goal_pose");
            return;
        }

        mbot_interfaces::msg::Pose2DArray path;
        bool success = planner_.planPath(dist_grid_, robot_pose_.value(), goal_pose_.value(), path);

        if (!success || path.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to plan path from (%.2f, %.2f) to (%.2f, %.2f)",
                        robot_pose_->x, robot_pose_->y, goal_pose_->x, goal_pose_->y);
            return;
        }

        publishWaypoints(path);
        publishPath(path);
    }

    void publishWaypoints(const mbot_interfaces::msg::Pose2DArray& path)
    {
        waypoints_pub_->publish(path);
    }

    void publishPath(const mbot_interfaces::msg::Pose2DArray& path)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        for (const auto& pose2d : path.poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = pose2d.x;
            pose_stamped.pose.position.y = pose2d.y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation = quaternionFromYaw(pose2d.theta);

            path_msg.poses.push_back(pose_stamped);
        }

        path_pub_->publish(path_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
