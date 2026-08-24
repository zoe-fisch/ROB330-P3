#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
            return;
        }

        // Select the next frontier goal
        auto goal = frontier_explorer_.selectGoal(robot_pose, dist_grid_);
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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
