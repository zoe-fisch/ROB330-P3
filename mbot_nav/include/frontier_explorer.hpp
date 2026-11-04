#pragma once

#include <vector>
#include <optional>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "obstacle_distance_grid.hpp"

class FrontierExplorer
{
public:
    FrontierExplorer() = default;

    std::optional<geometry_msgs::msg::Pose2D> selectGoal(
        const geometry_msgs::msg::Pose2D &robot_pose,
        const ObstacleDistanceGrid &dist_grid);

    // Access last-detected frontier data for visualization
    const std::vector<geometry_msgs::msg::Point>& getLastRawFrontiers() const { return last_raw_frontiers_; }
    const std::vector<geometry_msgs::msg::Point>& getLastClusteredFrontiers() const { return last_clustered_frontiers_; }

    // Minimum distance (meters) to consider a frontier
    double min_frontier_distance = 0.1;

    // Clearance (meters) required around robot positions from obstacles
    double robot_clearance = 0.15;

    // Distance (meters) for clustering nearby frontiers
    double max_cluster_link_distance = 0.15;

private:
    // Detects frontier cells and returns their world coordinates
    std::vector<geometry_msgs::msg::Point> detectFrontiers(
        int robot_gx, int robot_gy,
        const ObstacleDistanceGrid &dist_grid);

    // Clusters frontier points by distance threshold, returns centroids
    std::vector<geometry_msgs::msg::Point> clusterFrontiers(
        const std::vector<geometry_msgs::msg::Point> &frontiers);

    bool isFrontierCell(int x, int y, const ObstacleDistanceGrid &dist_grid);

    geometry_msgs::msg::Point gridToWorld(
        int x, int y,
        const ObstacleDistanceGrid &dist_grid);

    // Cached for visualization
    std::vector<geometry_msgs::msg::Point> last_raw_frontiers_;
    std::vector<geometry_msgs::msg::Point> last_clustered_frontiers_;
};
