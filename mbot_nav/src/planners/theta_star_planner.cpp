#include "planners/theta_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

float ThetaStarPlanner::heuristic(int x1, int y1, int x2, int y2)
{
    return std::hypot(x2 - x1, y2 - y1);
}

// The endpoints are grid-cell centers expressed in world coordinates.
bool ThetaStarPlanner::isLineFree(const ObstacleDistanceGrid& grid,
                                  float x0, float y0, float x1, float y1)
{
    const auto origin = grid.getOrigin().position;
    const double res = grid.getResolution();

    int x = std::floor((x0 - origin.x) / res);
    int y = std::floor((y0 - origin.y) / res);
    const int end_x = std::floor((x1 - origin.x) / res);
    const int end_y = std::floor((y1 - origin.y) / res);

    auto clear = [&](int cx, int cy) {
        return grid.isCellInGrid(cx, cy)
            && grid.getOccupancy(cx, cy) == 0
            && grid.getDistance(cx, cy) >= inflation_radius_;
    };

    const int dx = std::abs(end_x - x);
    const int dy = std::abs(end_y - y);
    const int sx = (end_x > x) - (end_x < x);
    const int sy = (end_y > y) - (end_y < y);
    int steps_x = 0;
    int steps_y = 0;

    while (clear(x, y)) {
        if (x == end_x && y == end_y) {
            return true;
        }

        // Determine which grid boundary the segment crosses next.
        const auto cross_x = (2LL * steps_x + 1) * dy;
        const auto cross_y = (2LL * steps_y + 1) * dx;

        // At an exact corner, check both side cells before moving diagonally.
        if (cross_x == cross_y && (!clear(x + sx, y) || !clear(x, y + sy))) {
            return false;
        }
        if (cross_x <= cross_y) {
            x += sx;
            ++steps_x;
        }
        if (cross_y <= cross_x) {
            y += sy;
            ++steps_y;
        }
    }

    return false;
}

std::vector<std::pair<int, int>> ThetaStarPlanner::backtraceWaypoints(
    const std::unordered_map<int, int>& came_from, int idx, int width)
{
    std::vector<std::pair<int, int>> cells;

    while (true) {
        cells.emplace_back(idx % width, idx / width);
        const int parent = came_from.at(idx);
        if (parent == idx) {
            break;
        }
        idx = parent;
    }

    std::reverse(cells.begin(), cells.end());
    return cells;
}

bool ThetaStarPlanner::planPath(const ObstacleDistanceGrid& grid,
                                const geometry_msgs::msg::Pose2D& start,
                                const geometry_msgs::msg::Pose2D& goal,
                                mbot_interfaces::msg::Pose2DArray& path)
{
    path.poses.clear();

    const double res = grid.getResolution();
    const int width = grid.getWidth();
    if (res <= 0 || width <= 0 || grid.getHeight() <= 0) {
        return false;
    }

    const auto origin = grid.getOrigin().position;
    const int sx = std::floor((start.x - origin.x) / res);
    const int sy = std::floor((start.y - origin.y) / res);
    const int gx = std::floor((goal.x - origin.x) / res);
    const int gy = std::floor((goal.y - origin.y) / res);

    auto worldX = [&](int x) { return origin.x + (x + 0.5) * res; };
    auto worldY = [&](int y) { return origin.y + (y + 0.5) * res; };
    auto visible = [&](int x, int y, int nx, int ny) {
        return isLineFree(grid, worldX(x), worldY(y), worldX(nx), worldY(ny));
    };

    if (!grid.isCellInGrid(sx, sy) || !grid.isCellInGrid(gx, gy)
        || !visible(sx, sy, sx, sy) || !visible(gx, gy, gx, gy)) {
        return false;
    }

    const int start_idx = toIndex(sx, sy, width);
    const int goal_idx = toIndex(gx, gy, width);

    std::priority_queue<ThetaStarNode, std::vector<ThetaStarNode>,
                        std::greater<>> open;
    std::unordered_map<int, float> cost{{start_idx, 0.0f}};
    std::unordered_map<int, int> parent{{start_idx, start_idx}};

    open.push({sx, sy, 0.0f, heuristic(sx, sy, gx, gy)});

    const int offsets[][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    while (!open.empty()) {
        const auto current = open.top();
        open.pop();

        const int idx = toIndex(current.x, current.y, width);
        if (current.cost > cost.at(idx)) {
            continue;
        }

        if (idx == goal_idx) {
            if (start_idx != goal_idx) {
                for (auto [x, y] : backtraceWaypoints(parent, idx, width)) {
                    geometry_msgs::msg::Pose2D pose;
                    pose.x = worldX(x);
                    pose.y = worldY(y);
                    path.poses.push_back(pose);
                }
            }

            // Finish at the requested position and heading within the goal cell.
            if (!path.poses.empty() && path.poses.back().x == goal.x
                                    && path.poses.back().y == goal.y) {
                path.poses.back() = goal;
            } else {
                path.poses.push_back(goal);
            }

            for (std::size_t i = 1; i < path.poses.size(); ++i) {
                auto& prev = path.poses[i - 1];
                const auto& next = path.poses[i];
                prev.theta = std::atan2(next.y - prev.y, next.x - prev.x);
            }
            return true;
        }

        const int parent_idx = parent.at(idx);
        const int px = parent_idx % width;
        const int py = parent_idx / width;

        for (const auto& offset : offsets) {
            const int nx = current.x + offset[0];
            const int ny = current.y + offset[1];
            if (!grid.isCellInGrid(nx, ny)) {
                continue;
            }

            // Theta*: connect the neighbor directly to the current cell's
            // parent when the entire shortcut has enough clearance.
            int from = idx;
            float candidate;
            if (visible(px, py, nx, ny)) {
                from = parent_idx;
                candidate = cost.at(parent_idx) + heuristic(px, py, nx, ny);
            } else if (visible(current.x, current.y, nx, ny)) {
                candidate = current.cost + heuristic(current.x, current.y, nx, ny);
            } else {
                continue;
            }

            const int neighbor_idx = toIndex(nx, ny, width);
            const auto old = cost.find(neighbor_idx);
            if (old != cost.end() && candidate >= old->second) {
                continue;
            }

            cost[neighbor_idx] = candidate;
            parent[neighbor_idx] = from;
            open.push({nx, ny, candidate, candidate + heuristic(nx, ny, gx, gy)});
        }
    }

    return false;
}