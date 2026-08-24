#include "frontier_explorer.hpp"
#include <queue>
#include <map>
#include <limits>
#include "rclcpp/rclcpp.hpp"

std::optional<geometry_msgs::msg::Pose2D> FrontierExplorer::selectGoal(
    const geometry_msgs::msg::Pose2D &robot_pose,
    const ObstacleDistanceGrid &dist_grid)
{
    // Convert robot pose to grid coordinates
    int robot_gx = static_cast<int>((robot_pose.x - dist_grid.getOrigin().position.x) / dist_grid.getResolution());
    int robot_gy = static_cast<int>((robot_pose.y - dist_grid.getOrigin().position.y) / dist_grid.getResolution());

    // Detect frontier cells
    auto raw_frontiers = detectFrontiers(robot_gx, robot_gy, dist_grid);

    if (raw_frontiers.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("frontier_explorer"), "No frontiers found");
        return std::nullopt;
    }

    RCLCPP_INFO(rclcpp::get_logger("frontier_explorer"), "Found %zu frontier points", raw_frontiers.size());

    // Cluster frontiers by distance
    auto clustered = clusterFrontiers(raw_frontiers);

    // Pick the closest frontier cluster
    double closest_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point best_frontier;

    for (const auto& centroid : clustered) {
        double dx = centroid.x - robot_pose.x;
        double dy = centroid.y - robot_pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Skip if too close to robot
        if (distance < min_frontier_distance) {
            continue;
        }

        if (distance < closest_distance) {
            closest_distance = distance;
            best_frontier = centroid;
        }
    }

    // Check if we found a valid frontier
    if (closest_distance == std::numeric_limits<double>::max()) {
        RCLCPP_INFO(rclcpp::get_logger("frontier_explorer"), "No valid frontiers (all too close)");
        return std::nullopt;
    }

    // Calculate heading toward the frontier
    double dx = best_frontier.x - robot_pose.x;
    double dy = best_frontier.y - robot_pose.y;
    double approach_angle = std::atan2(dy, dx);

    geometry_msgs::msg::Pose2D goal;
    goal.x = best_frontier.x;
    goal.y = best_frontier.y;
    goal.theta = approach_angle;

    return goal;
}

std::vector<geometry_msgs::msg::Point> FrontierExplorer::detectFrontiers(
    int robot_gx, int robot_gy,
    const ObstacleDistanceGrid &dist_grid)
{
    int width = dist_grid.getWidth();
    int height = dist_grid.getHeight();
    std::vector<geometry_msgs::msg::Point> frontier_points;

    // TODO: How would you define and find frontiers
    // use isFrontierCell()
    (void)robot_gx; (void)robot_gy;
    (void) width; (void)height;

    return frontier_points;
}

std::vector<geometry_msgs::msg::Point> FrontierExplorer::clusterFrontiers(
    const std::vector<geometry_msgs::msg::Point> &frontiers)
{
    if (frontiers.empty()) return {};
    
    std::vector<geometry_msgs::msg::Point> centroids;

    // TODO: we cluster frontiers to centroids and return the vector
    geometry_msgs::msg::Point centroid;
    centroids.push_back(centroid);

    return centroids;
}

bool FrontierExplorer::isFrontierCell(int x, int y, const ObstacleDistanceGrid &dist_grid)
{
    // Must be in bounds
    if (!dist_grid.isCellInGrid(x, y)) {
        return false;
    }

    // Must be a free cell (occupancy = 0)
    if (dist_grid.getOccupancy(x, y) != 0) {
        return false;
    }

    // Must be adjacent to unknown cell (8-connectivity)
    const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};

    for (int i = 0; i < 8; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (dist_grid.isCellInGrid(nx, ny)) {
            if (dist_grid.getOccupancy(nx, ny) == -1) {
                return true;  // Adjacent to unknown
            }
        }
    }

    return false;
}

geometry_msgs::msg::Point FrontierExplorer::gridToWorld(
    int x, int y,
    const ObstacleDistanceGrid &dist_grid)
{
    geometry_msgs::msg::Point pt;
    pt.x = dist_grid.getOrigin().position.x + (x + 0.5) * dist_grid.getResolution();
    pt.y = dist_grid.getOrigin().position.y + (y + 0.5) * dist_grid.getResolution();
    pt.z = 0.0;
    return pt;
}
