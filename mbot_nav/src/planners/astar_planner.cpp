#include "planners/astar_planner.hpp"

#include <cmath>
#include <limits>

float AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

bool AStarPlanner::isLineFree(const ObstacleDistanceGrid& dist_grid, float x0, float y0, float x1, float y1)
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
        if (!dist_grid.isCellInGrid(grid_x, grid_y) || dist_grid.getDistance(grid_x, grid_y) < 0.15f) {
            return false;
        }
    }
    return true;
}

std::vector<std::pair<int, int>> AStarPlanner::backtraceWaypoints(
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

bool AStarPlanner::planPath(const ObstacleDistanceGrid& dist_grid,
                            const geometry_msgs::msg::Pose2D& start,
                            const geometry_msgs::msg::Pose2D& goal,
                            mbot_interfaces::msg::Pose2DArray& path) {
    path.poses.clear();

    mbot_interfaces::msg::Pose2DArray raw_path;
    
    const float res = dist_grid.getResolution();
    const float origin_x  = dist_grid.getOrigin().position.x;
    const float origin_y  = dist_grid.getOrigin().position.y;
    const int width = dist_grid.getWidth();

    // start and goal in grid coordinates
    int start_x = static_cast<int>((start.x - origin_x) / res);
    int start_y = static_cast<int>((start.y - origin_y) / res);
    int goal_x = static_cast<int>((goal.x - origin_x) / res);
    int goal_y = static_cast<int>((goal.y - origin_y) / res);

    // Min-priority queue
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<>> open;
    // Hash tables, similar to python's dict
    std::unordered_map<int, float> cost_so_far;
    std::unordered_map<int, int> came_from;

    int start_idx = toIndex(start_x, start_y, width);
    int goal_idx = toIndex(goal_x, goal_y, width);

    open.push({start_x, start_y, 0.0f, heuristic(start_x, start_y, goal_x, goal_y)});
    cost_so_far[start_idx] = 0.0f;
    came_from[start_idx] = start_idx;

    struct Offset { int x; int y; };
    const std::vector<Offset> connected_neighbors = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},  // Cardinal directions
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1} // Diagonals
    };

    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        int curr_idx = toIndex(current.x, current.y, width);
        if (curr_idx == goal_idx) {
            // goal reached, generate the path
            auto waypoints_idx = backtraceWaypoints(came_from, goal_idx, width);
            for (size_t i = 0; i < waypoints_idx.size(); ++i) {
                const auto& [x, y] = waypoints_idx[i];
                // generate world frame pose
                geometry_msgs::msg::Pose2D pose;
                pose.x = (x + 0.5f) * res + origin_x;
                pose.y = (y + 0.5f) * res + origin_y;
                // TODO: how do we determine the heading?
                // pose.theta = ?
                // fix 
                if (i + 1 < waypoints_idx.size()) {
                    const auto& [next_x, next_y] = waypoints_idx[i + 1]; // fix
                    pose.theta = std::atan2(next_y - y, next_x - x);
                } 
                else {
                    pose.theta = goal.theta;  
                }
                raw_path.poses.push_back(pose);
            }
            path = smoothPath(dist_grid, raw_path);
            return true;
        }

        for (const auto& offset : connected_neighbors) {
            int nx = current.x + offset.x;
            int ny = current.y + offset.y;

            // TODO: check neighbors, calcualte costs, then update cost_so_far, came_from, and open
            // hints: utilize dist_grid.getDistance() and dist_grid.getOccupancy()
            
            double cost;
            if (!dist_grid.isCellInGrid(nx, ny)) {
                continue;
            }

            if (dist_grid.getDistance(nx, ny) < 0.15f) {
                continue;
            }

            // fix
            if (offset.x != 0 && offset.y != 0) {
                if (dist_grid.getDistance(current.x + offset.x, current.y) < 0.15f ||
                    dist_grid.getDistance(current.x, current.y + offset.y) < 0.15f) {
                    continue;
                }
            }

            int neighbor = toIndex(nx, ny, width);

            if (offset.x != 0 && offset.y != 0) {
                cost = 1.4;
            } 
            else {
                cost = 1;
            }
            double updated_cost = cost_so_far[curr_idx] + cost;
            bool not_seen_yet = cost_so_far.count(neighbor) == 0;
            if (not_seen_yet || updated_cost < cost_so_far[neighbor]) {
                cost_so_far[neighbor] = updated_cost;
                came_from[neighbor] = curr_idx;
                open.push({nx, ny, updated_cost, heuristic(nx, ny, goal_x, goal_y)});
            }
        
        }
    }

    return false;
}

// fix 
mbot_interfaces::msg::Pose2DArray AStarPlanner::smoothPath(const ObstacleDistanceGrid& dist_grid, const mbot_interfaces::msg::Pose2DArray& raw_path) {
    mbot_interfaces::msg::Pose2DArray pruned;
    if (raw_path.poses.empty()) {
        return pruned;
    }
    pruned.poses.push_back(raw_path.poses[0]);
    size_t current = 0;
    while (current + 1 < raw_path.poses.size()) {
        size_t farthest = current + 1;
        for (size_t candidate = current + 1; candidate < raw_path.poses.size(); ++candidate) {
            const geometry_msgs::msg::Pose2D& from = raw_path.poses[current];
            const geometry_msgs::msg::Pose2D& to = raw_path.poses[candidate];
            if (isLineFree(dist_grid, from.x, from.y, to.x, to.y)) {
                farthest = candidate;
            }
        }
        pruned.poses.push_back(raw_path.poses[farthest]);
        current = farthest;
    }
    return pruned;
}