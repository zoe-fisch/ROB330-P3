#include "obstacle_distance_grid.hpp"
#include <limits>
#include <cmath>

ObstacleDistanceGrid::ObstacleDistanceGrid()
: width_(0), height_(0), resolution_(0.05f)
{}

void ObstacleDistanceGrid::computeDistFromMap(const nav_msgs::msg::OccupancyGrid &map)
{
    width_ = map.info.width;
    height_ = map.info.height;
    resolution_ = map.info.resolution;
    origin_ = map.info.origin;

    occupancy_ = map.data;
    distances_.assign(width_ * height_, std::numeric_limits<float>::max());

    computeDistances();
}

bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0 && x < width_ && y >= 0 && y < height_);
}

float ObstacleDistanceGrid::getDistance(int x, int y) const
{
    if (!isCellInGrid(x, y)) return std::numeric_limits<float>::max();
    return distances_[y * width_ + x];
}

int8_t ObstacleDistanceGrid::getOccupancy(int x, int y) const
{
    if (!isCellInGrid(x, y)) return -1;  // unknown if out of bounds
    return occupancy_[y * width_ + x];
}

void ObstacleDistanceGrid::computeDistances()
{
    std::queue<GridCell> queue;

    // Step 1: Initialize queue with all occupied cells (value >= 50)
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = y * width_ + x;
            if (occupancy_[idx] >= 50) {
                distances_[idx] = 0.0f;
                queue.emplace(x, y);
            }
        }
    }

    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = {0, 0, -1, 1};

    while (!queue.empty()) {
        GridCell cell = queue.front();
        queue.pop();

        float nextDistance = getDistance(cell.x, cell.y) + resolution_;

        for (int i = 0; i < 4; ++i) {
            int nx = cell.x + dx[i];
            int ny = cell.y + dy[i];
            if (!isCellInGrid(nx, ny)) continue;

            int idx = ny * width_ + nx;
            if (nextDistance < distances_[idx]) {
                distances_[idx] = nextDistance;
                queue.emplace(nx, ny);
            }
        }
    }
}

