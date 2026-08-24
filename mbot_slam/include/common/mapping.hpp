#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <limits>
#include <cstdint>

class OccupancyGrid {
public:
    OccupancyGrid(float width_m, float height_m, float resolution, float origin_x, float origin_y);

    void markCellOccupied(int x, int y);
    void markCellFree(int x, int y);

    // Convert log-odds to probability percentage [0-100]
    // -1 for unknown cells (ROS format)
    std::vector<int8_t> getOccupancyGrid() const;

    std::pair<int, int> worldToGrid(float x, float y) const;
    int gridToIndex(int x, int y) const;

    float getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getOriginX() const { return origin_x_; }
    float getOriginY() const { return origin_y_; }

private:
    float resolution_;
    float origin_x_, origin_y_;
    int width_, height_;
    std::vector<float> log_odds_;  // flat row-major grid; NaN = unknown

    const float kHitLogOdds = 0.85f;  
    const float kMissLogOdds = -0.4f;
    const float kMinLogOdds = -2.0f;
    const float kMaxLogOdds = 3.5f;
};

// Bresenham-style ray tracing from sensor origin to hit
std::vector<std::pair<int, int>> bresenhamRayTrace(
    float origin_x, float origin_y, float theta, float range, const OccupancyGrid& grid);