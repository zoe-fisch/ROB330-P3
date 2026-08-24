#include "planners/theta_star_planner.hpp"

#include <cmath>
#include <limits>

float ThetaStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

bool ThetaStarPlanner::isLineFree(const ObstacleDistanceGrid& dist_grid,
                                   float x0, float y0, float x1, float y1)
{
    const float step = dist_grid.getResolution() * 0.5f;
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float dist = std::hypot(dx, dy);
    const float angle = std::atan2(dy, dx);

    for (float d = 0.0f; d <= dist; d += step) {
        float x = x0 + std::cos(angle) * d;
        float y = y0 + std::sin(angle) * d;
        int grid_x = static_cast<int>((x - dist_grid.getOrigin().position.x) / dist_grid.getResolution());
        int grid_y = static_cast<int>((y - dist_grid.getOrigin().position.y) / dist_grid.getResolution());
        if (!dist_grid.isCellInGrid(grid_x, grid_y) || dist_grid.getDistance(grid_x, grid_y) < 0.2f) {
            return false;
        }
    }
    return true;
}

std::vector<std::pair<int, int>> ThetaStarPlanner::backtraceWaypoints(
    const std::unordered_map<int, int>& came_from, int goal_idx, int width)
{
    std::vector<std::pair<int, int>> path;
    int current = goal_idx;

    while (true) {
        path.emplace_back(current % width, current / width);
        int next = came_from.at(current);
        if (next == current) break; // We found the start node!
        current = next;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

bool ThetaStarPlanner::planPath(const ObstacleDistanceGrid& dist_grid,
                                 const geometry_msgs::msg::Pose2D& start,
                                 const geometry_msgs::msg::Pose2D& goal,
                                 mbot_interfaces::msg::Pose2DArray& path)
{
    path.poses.clear();

    const float res = dist_grid.getResolution();
    const float origin_x  = dist_grid.getOrigin().position.x;
    const float origin_y  = dist_grid.getOrigin().position.y;
    const int width = dist_grid.getWidth();

    int start_x = static_cast<int>((start.x - origin_x) / res);
    int start_y = static_cast<int>((start.y - origin_y) / res);
    int goal_x  = static_cast<int>((goal.x - origin_x) / res);
    int goal_y  = static_cast<int>((goal.y - origin_y) / res);

    // Min-priority queue
    std::priority_queue<ThetaStarNode, std::vector<ThetaStarNode>, std::greater<>> open;
    // Hash tables, similar to python's dict
    std::unordered_map<int, float> cost_so_far;
    std::unordered_map<int, int> came_from;

    int start_idx = toIndex(start_x, start_y, width);
    int goal_idx  = toIndex(goal_x, goal_y, width);

    open.push({start_x, start_y, 0.0f, heuristic(start_x, start_y, goal_x, goal_y)});
    cost_so_far[start_idx] = 0.0f;
    came_from[start_idx]   = start_idx;

    struct Offset { int x; int y; };
    const std::vector<Offset> connected_neighbors = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},  // Cardinal directions
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1} // Diagonals
    };

    while (!open.empty()) {
        ThetaStarNode current = open.top();
        open.pop();

        int curr_idx = toIndex(current.x, current.y, width);
        if (curr_idx == goal_idx) {
            // goal reached, generate the path
            auto waypoints_idx = backtraceWaypoints(came_from, goal_idx, width);
            for (size_t i = 0; i < waypoints_idx.size(); ++i) {
                const auto& [x, y] = waypoints_idx[i];
                // generate world frame pose
                geometry_msgs::msg::Pose2D pose;
                pose.x = x * res + origin_x;
                pose.y = y * res + origin_y;
                // TODO: how do we determine the heading?
                // pose.theta = ?
                path.poses.push_back(pose);
            }
            return true;
        }

        // TODO: we need to track grandparents in theta star

        // Check 8 neighbors
        for (const auto& offset : connected_neighbors) {
            int nx = current.x + offset.x;
            int ny = current.y + offset.y;

            // TODO: check neighbors, calcualte costs, then update cost_so_far, and open
            // Update came_from also checking grandparents, this is the biggest difference between astar and theta star
            // hints: utilize dist_grid.getDistance() and dist_grid.getOccupancy()
            (void)nx; (void)ny;
        }
    }

    return false;
}
