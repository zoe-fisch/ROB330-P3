#include "common/mapping.hpp"
#include <cmath>
#include <algorithm>

OccupancyGrid::OccupancyGrid(float width_m, float height_m, float resolution, float origin_x, float origin_y)
: resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y)
{
    width_ = static_cast<int>(width_m / resolution);
    height_ = static_cast<int>(height_m / resolution);
    log_odds_.resize(width_ * height_, std::numeric_limits<float>::quiet_NaN());
}

void OccupancyGrid::markCellOccupied(int x, int y)
{
    int idx = gridToIndex(x, y);
    if (idx >= 0 && static_cast<size_t>(idx) < log_odds_.size())
    {
        if (std::isnan(log_odds_[idx])) log_odds_[idx] = 0.0f;
        // TODO: how to assign log odds values to occupied cells?
        // log_odds_[idx] = ???
        // kHitLogOdds defined in mapping.hpp, you need to tune the parameters there
        // Why we need kMinLogOdds and kMaxLogOdds? <- think about it and tune these values
    }
}

void OccupancyGrid::markCellFree(int x, int y)
{
    int idx = gridToIndex(x, y);
    if (idx >= 0 && static_cast<size_t>(idx) < log_odds_.size())
    {
        if (std::isnan(log_odds_[idx])) log_odds_[idx] = 0.0f;
        // TODO: how to assign log odds values to free cells?
        // log_odds_[idx] = ???
        // kMissLogOdds defined in mapping.hpp, you need to tune the parameters there
        // Why we need kMinLogOdds and kMaxLogOdds? <- think about it and tune these values
    }
}

std::vector<int8_t> OccupancyGrid::getOccupancyGrid() const
{
    std::vector<int8_t> result;
    result.reserve(log_odds_.size());

    for (float log_odds : log_odds_)
    {
        if (std::isnan(log_odds)) {
            result.push_back(-1);  // unknown
        } else {
            float prob = 1.0f - 1.0f / (1.0f + std::exp(log_odds));
            result.push_back(static_cast<int8_t>(std::round(prob * 100.0f)));
        }
    }

    return result;
}

std::pair<int, int> OccupancyGrid::worldToGrid(float x, float y) const {
    return {
        static_cast<int>((x - origin_x_) / resolution_),
        static_cast<int>((y - origin_y_) / resolution_)
    };
}

int OccupancyGrid::gridToIndex(int x, int y) const
{
    if (x < 0 || y < 0 || x >= width_ || y >= height_)
        return -1;
    return y * width_ + x;
}

std::vector<std::pair<int, int>> bresenhamRayTrace(
    float origin_x, float origin_y, float theta, float range, const OccupancyGrid& grid)
{
    std::vector<std::pair<int, int>> cells;

    // Start and end point in world
    float end_x = origin_x + range * std::cos(theta);
    float end_y = origin_y + range * std::sin(theta);

    // Convert to grid coordinates
    auto [x0, y0] = grid.worldToGrid(origin_x, origin_y);
    auto [x1, y1] = grid.worldToGrid(end_x, end_y);
    
    // TODO: Bresenham's algorithm
    // Implement the algorithm here, x0 y0 is the origin of the lidar ray
    // x1 y1 is the endpoint of the lidar ray
    // Hint: utilize the helper function here, such as gridToIndex

    (void)x0; (void)y0; (void)x1; (void)y1;
    
    
    return cells;
}