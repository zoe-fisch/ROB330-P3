#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mbot_interfaces/msg/pose2_d_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

// To switch controllers, change the include and the type of Controller below.
#include "controllers/rtr_controller.hpp"
// #include "controllers/pure_pursuit_controller.hpp"
// #include "controllers/example_controller.hpp"

using Controller = RTRController;
// using Controller = PurePursuitController;
// using Controller = ExampleController;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        this->declare_parameter<bool>("use_localization", false);
        use_localization_ = this->get_parameter("use_localization").as_bool();

        if (use_localization_) {
            RCLCPP_INFO(get_logger(), "Using LOCALIZATION mode (TF map->base_footprint)");
            tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        } else {
            RCLCPP_INFO(get_logger(), "Using ODOMETRY mode (/odom topic)");
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/odom", rclcpp::SensorDataQoS(),
                [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    current_x_     = msg->pose.pose.position.x;
                    current_y_     = msg->pose.pose.position.y;
                    current_theta_ = yawFromQuaternion(msg->pose.pose.orientation);
                });
        }

        waypoints_sub_ = create_subscription<mbot_interfaces::msg::Pose2DArray>(
            "/waypoints", 10,
            [this](const mbot_interfaces::msg::Pose2DArray::SharedPtr msg) {
                controller_.setPlan(msg->poses, current_x_, current_y_);
                RCLCPP_INFO(get_logger(), "Received %zu waypoints", msg->poses.size());
            });

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { timerCallback(); });

        RCLCPP_INFO(get_logger(), "ControllerNode initialized");
    }

private:
    bool   use_localization_{false};
    double current_x_{0}, current_y_{0}, current_theta_{0};

    Controller controller_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
    rclcpp::Subscription<mbot_interfaces::msg::Pose2DArray>::SharedPtr  waypoints_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr                                        timer_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void timerCallback()
    {
        if (use_localization_) {
            try {
                auto tf = tf_buffer_->lookupTransform("map", "base_footprint", rclcpp::Time(0));
                current_x_     = tf.transform.translation.x;
                current_y_     = tf.transform.translation.y;
                current_theta_ = yawFromQuaternion(tf.transform.rotation);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Cannot get map->base_footprint: %s", ex.what());
                return;
            }
        }

        auto cmd = controller_.computeVelocityCommands(current_x_, current_y_, current_theta_);
        cmd_vel_pub_->publish(cmd);
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
