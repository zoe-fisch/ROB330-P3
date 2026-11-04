#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "frontier_explorer.hpp"
#include "obstacle_distance_grid.hpp"
#include "utils.hpp"

#include <memory>
#include <optional>
#include <cmath>

class ExplorationNode : public rclcpp::Node
{
public:
    ExplorationNode()
    : Node("exploration_node")
    {
        // Publishers
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        frontier_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/frontier_markers", 10);

        // Subscriptions
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&ExplorationNode::mapCallback, this, std::placeholders::_1));

        // Timer to run frontier selection
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0),
            std::bind(&ExplorationNode::timerCallback, this));

        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    // Timer for periodic frontier selection
    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_pub_;

    // Data
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::optional<geometry_msgs::msg::Pose2D> current_goal_;

    // Frontier detection
    FrontierExplorer frontier_explorer_;
    ObstacleDistanceGrid dist_grid_;

    // TF support
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Exploration state
    std::optional<geometry_msgs::msg::Pose2D> origin_pose_;
    bool is_returning_ = false;
    int no_frontier_count_ = 0;       // Counter for consecutive "no frontier" results
    int no_frontier_retry_ = 10;      // Give it N retries before returning to origin
    double goal_reached_threshold_ = 0.15;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = msg;
        dist_grid_.computeDistFromMap(*msg);
    }

    void timerCallback()
    {
        if (!latest_map_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for map...");
            return;
        }

        geometry_msgs::msg::Pose2D robot_pose;
        try {
            geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);

            robot_pose.x     = tf.transform.translation.x;
            robot_pose.y     = tf.transform.translation.y;
            robot_pose.theta = yawFromQuaternion(tf.transform.rotation);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        // Latch origin on first successful pose
        if (!origin_pose_) {
            origin_pose_ = robot_pose;
            RCLCPP_INFO(this->get_logger(), "Origin: (%.2f, %.2f)", origin_pose_->x, origin_pose_->y);
            return;
        }

        // Check if current goal was reached
        if (current_goal_) {
            double dx = robot_pose.x - current_goal_->x;
            double dy = robot_pose.y - current_goal_->y;
            if (std::sqrt(dx * dx + dy * dy) <= goal_reached_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                current_goal_.reset();
            } else {
                publishFrontierMarkers();
                return;  // Still navigating — navigation_node replans on its own timer
            }
        }

        // Return to origin once exploration is done
        if (is_returning_) {
            double dx = robot_pose.x - origin_pose_->x;
            double dy = robot_pose.y - origin_pose_->y;
            if (std::sqrt(dx * dx + dy * dy) <= goal_reached_threshold_) {
                RCLCPP_INFO(this->get_logger(), "Returned to origin. Exploration complete.");
            }
            publishFrontierMarkers();
            return;
        }

        // Select the next frontier goal
        auto goal = frontier_explorer_.selectGoal(robot_pose, dist_grid_);
        publishFrontierMarkers();
        if (!goal) {
            no_frontier_count_++;
            RCLCPP_WARN(this->get_logger(), "No frontier found (%d/%d).",
                        no_frontier_count_, no_frontier_retry_);
            if (no_frontier_count_ >= no_frontier_retry_) {
                RCLCPP_INFO(this->get_logger(), "Exploration done. Returning to origin.");
                current_goal_    = origin_pose_.value();
                is_returning_    = true;
                no_frontier_count_ = 0;
                publishGoal(current_goal_.value());
            }
            return;
        }

        no_frontier_count_ = 0;
        current_goal_ = goal.value();
        RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)",
                    current_goal_->x, current_goal_->y);
        publishGoal(current_goal_.value());
    }

    void publishGoal(const geometry_msgs::msg::Pose2D& goal)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = goal.x;
        msg.pose.position.y = goal.y;
        msg.pose.position.z = 0.0;
        msg.pose.orientation = quaternionFromYaw(goal.theta);
        goal_pub_->publish(msg);
    }

    void publishFrontierMarkers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Marker 0: all raw frontier cells as a POINTS cloud (cyan)
        {
            visualization_msgs::msg::Marker raw_marker;
            raw_marker.header.frame_id = "map";
            raw_marker.header.stamp = this->now();
            raw_marker.ns = "frontier_raw";
            raw_marker.id = 0;
            raw_marker.type = visualization_msgs::msg::Marker::POINTS;
            raw_marker.action = visualization_msgs::msg::Marker::ADD;
            raw_marker.pose.orientation.w = 1.0;
            raw_marker.scale.x = 0.05;
            raw_marker.scale.y = 0.05;
            raw_marker.color.r = 0.0f;
            raw_marker.color.g = 1.0f;
            raw_marker.color.b = 1.0f;
            raw_marker.color.a = 0.8f;
            raw_marker.lifetime = rclcpp::Duration(2, 0);  // expire after 2s so stale frontiers clear

            for (const auto& pt : frontier_explorer_.getLastRawFrontiers()) {
                raw_marker.points.push_back(pt);
            }
            marker_array.markers.push_back(raw_marker);
        }

        // Marker 1: clustered frontier centroids as green spheres
        {
            visualization_msgs::msg::Marker cluster_marker;
            cluster_marker.header.frame_id = "map";
            cluster_marker.header.stamp = this->now();
            cluster_marker.ns = "frontier_clusters";
            cluster_marker.id = 1;
            cluster_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            cluster_marker.action = visualization_msgs::msg::Marker::ADD;
            cluster_marker.pose.orientation.w = 1.0;
            cluster_marker.scale.x = 0.12;
            cluster_marker.scale.y = 0.12;
            cluster_marker.scale.z = 0.12;
            cluster_marker.color.r = 0.0f;
            cluster_marker.color.g = 1.0f;
            cluster_marker.color.b = 0.0f;
            cluster_marker.color.a = 1.0f;
            cluster_marker.lifetime = rclcpp::Duration(2, 0);

            for (const auto& pt : frontier_explorer_.getLastClusteredFrontiers()) {
                cluster_marker.points.push_back(pt);
            }
            marker_array.markers.push_back(cluster_marker);
        }

        // Marker 2: current goal as a large yellow sphere
        if (current_goal_) {
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = "map";
            goal_marker.header.stamp = this->now();
            goal_marker.ns = "frontier_goal";
            goal_marker.id = 2;
            goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            goal_marker.pose.position.x = current_goal_->x;
            goal_marker.pose.position.y = current_goal_->y;
            goal_marker.pose.position.z = 0.0;
            goal_marker.pose.orientation.w = 1.0;
            goal_marker.scale.x = 0.2;
            goal_marker.scale.y = 0.2;
            goal_marker.scale.z = 0.2;
            goal_marker.color.r = 1.0f;
            goal_marker.color.g = 1.0f;
            goal_marker.color.b = 0.0f;
            goal_marker.color.a = 1.0f;
            goal_marker.lifetime = rclcpp::Duration(2, 0);
            marker_array.markers.push_back(goal_marker);
        }

        frontier_pub_->publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
