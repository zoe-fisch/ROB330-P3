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
    last_raw_frontiers_ = raw_frontiers;

    if (raw_frontiers.empty()) {
        last_clustered_frontiers_.clear();  // Don't keep showing old clusters
        RCLCPP_INFO(rclcpp::get_logger("frontier_explorer"), "No frontiers found");
        return std::nullopt;
    }

    RCLCPP_INFO(rclcpp::get_logger("frontier_explorer"), "Found %zu frontier points", raw_frontiers.size());

    // Cluster frontiers by distance
    auto clustered = clusterFrontiers(raw_frontiers);
    last_clustered_frontiers_ = clustered;

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
    // This version scans the whole map.
    (void)robot_gx;
    (void)robot_gy;

    std::vector<geometry_msgs::msg::Point> frontier_points;

    for (int y = 0; y < dist_grid.getHeight(); ++y) {
        for (int x = 0; x < dist_grid.getWidth(); ++x) {
            if (isFrontierCell(x, y, dist_grid) &&
                dist_grid.getDistance(x, y) >= robot_clearance) {
                frontier_points.push_back(gridToWorld(x, y, dist_grid));
            }
        }
    }

    return frontier_points;
}

std::vector<geometry_msgs::msg::Point> FrontierExplorer::clusterFrontiers(
    const std::vector<geometry_msgs::msg::Point> &frontiers)
{
    std::vector<geometry_msgs::msg::Point> centroids;
    std::vector<bool> visited(frontiers.size(), false);

    for (std::size_t start = 0; start < frontiers.size(); ++start) {
        if (visited[start]) continue;

        std::queue<std::size_t> pending;
        pending.push(start);
        visited[start] = true;

        double sum_x = 0.0;
        double sum_y = 0.0;
        std::size_t count = 0;

        // Grow this cluster from each newly discovered member.
        while (!pending.empty()) {
            const auto current = pending.front();
            pending.pop();

            const auto &point = frontiers[current];
            sum_x += point.x;
            sum_y += point.y;
            ++count;

            for (std::size_t i = 0; i < frontiers.size(); ++i) {
                if (visited[i]) continue;

                const double dx = frontiers[i].x - point.x;
                const double dy = frontiers[i].y - point.y;

                if (std::hypot(dx, dy) <= max_cluster_link_distance) {
                    visited[i] = true;
                    pending.push(i);
                }
            }
        }

        geometry_msgs::msg::Point centroid;
        centroid.x = sum_x / count;
        centroid.y = sum_y / count;
        centroid.z = 0.0;
        centroids.push_back(centroid);
    }

    return centroids;
}

bool FrontierExplorer::isFrontierCell(int x, int y, const ObstacleDistanceGrid &dist_grid)
{
    if (!dist_grid.isFree(x, y)) {
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
