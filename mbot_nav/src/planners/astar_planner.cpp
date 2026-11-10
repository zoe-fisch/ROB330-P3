#include "planners/astar_planner.hpp"

#include <cmath>
#include <algorithm>
#include <functional>

float AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
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

    const double res = dist_grid.getResolution();
    const auto origin = dist_grid.getOrigin().position;
    const int width = dist_grid.getWidth();
    if (!(res > 0.0) || width <= 0 || dist_grid.getHeight() <= 0) return false;

    const int start_x = std::floor((start.x - origin.x) / res);
    const int start_y = std::floor((start.y - origin.y) / res);
    const int goal_x = std::floor((goal.x - origin.x) / res);
    const int goal_y = std::floor((goal.y - origin.y) / res);

    const auto visible = [&](int x, int y, int nx, int ny) {
        return dist_grid.isLineFree(x, y, nx, ny, inflation_radius_);
    };
    if (!visible(start_x, start_y, start_x, start_y) ||
        !visible(goal_x, goal_y, goal_x, goal_y)) {
        return false;
    }

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

        const int curr_idx = toIndex(current.x, current.y, width);
        if (current.cost > cost_so_far.at(curr_idx)) continue;

        if (curr_idx == goal_idx) {
            const auto cells = backtraceWaypoints(came_from, goal_idx, width);
            if (cells.size() > 1) {
                size_t current_cell = 0;
                while (true) {
                    const auto [x, y] = cells[current_cell];
                    geometry_msgs::msg::Pose2D pose;
                    pose.x = origin.x + (x + 0.5) * res;
                    pose.y = origin.y + (y + 0.5) * res;
                    path.poses.push_back(pose);
                    if (current_cell == cells.size() - 1) break;

                    size_t farthest = cells.size() - 1;
                    while (farthest > current_cell + 1 &&
                           !visible(x, y, cells[farthest].first, cells[farthest].second)) {
                        --farthest;
                    }
                    current_cell = farthest;
                }
            }

            if (!path.poses.empty() && path.poses.back().x == goal.x &&
                path.poses.back().y == goal.y) {
                path.poses.back() = goal;
            } else {
                path.poses.push_back(goal);
            }

            for (size_t i = 1; i < path.poses.size(); ++i) {
                auto& prev = path.poses[i - 1];
                const auto& next = path.poses[i];
                prev.theta = std::atan2(next.y - prev.y, next.x - prev.x);
            }
            return true;
        }

        for (const auto& offset : connected_neighbors) {
            const int nx = current.x + offset.x;
            const int ny = current.y + offset.y;
            if (!visible(current.x, current.y, nx, ny)) continue;

            const int neighbor = toIndex(nx, ny, width);
            const float candidate = current.cost + heuristic(current.x, current.y, nx, ny);
            const auto old = cost_so_far.find(neighbor);
            if (old != cost_so_far.end() && candidate >= old->second) continue;

            cost_so_far[neighbor] = candidate;
            came_from[neighbor] = curr_idx;
            open.push({nx, ny, candidate, candidate + heuristic(nx, ny, goal_x, goal_y)});
        }
    }

    return false;
}
