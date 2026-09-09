#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mbot_interfaces/msg/pose2_d_array.hpp>
#include <mbot_interfaces/msg/pid_debug.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>

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
        debug_pub_   = create_publisher<mbot_interfaces::msg::PIDDebug>("/debug_cmd_vel", 10);

        // CSV log. Rows are written as they happen (not buffered until shutdown),
        // so the file survives Ctrl-C or a hard kill.
        const char * home = std::getenv("HOME");
        const std::string default_log =
            std::string(home ? home : ".") + "/mbot_logs/pid_debug_log.csv";
        this->declare_parameter<std::string>("log_path", default_log);
        log_path_ = this->get_parameter("log_path").as_string();

        std::filesystem::create_directories(
            std::filesystem::path(log_path_).parent_path());
        log_file_.open(log_path_, std::ios::out | std::ios::trunc);
        if (log_file_.is_open()) {
            log_file_ << "time,x,y,theta,goal_x,goal_y,distance_error,angle_error,"
                         "lin_cmd,ang_cmd\n";
            log_file_.flush();
            RCLCPP_INFO(get_logger(), "Logging PID debug data to %s", log_path_.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Could not open log file %s", log_path_.c_str());
        }

        t0_ = this->now();

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
    rclcpp::Publisher<mbot_interfaces::msg::PIDDebug>::SharedPtr        debug_pub_;
    rclcpp::TimerBase::SharedPtr                                        timer_;
    rclcpp::Time                                                        t0_;
    std::string                                                         log_path_;
    std::ofstream                                                       log_file_;

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

        // The controller has no node of its own, so the debug data it recorded
        // this tick is published and logged from here.
        if (!controller_.hasDebug()) return;

        const auto & dbg = controller_.getLastDebug();
        debug_pub_->publish(dbg);

        if (log_file_.is_open()) {
            log_file_ << (this->now() - t0_).seconds() << ","
                      << current_x_ << "," << current_y_ << "," << current_theta_ << ","
                      << controller_.getDebugGoalX() << ","
                      << controller_.getDebugGoalY() << ","
                      << dbg.signed_distance_error << "," << dbg.signed_angle_error << ","
                      << cmd.linear.x << "," << cmd.angular.z << "\n";
            log_file_.flush();
        }
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
