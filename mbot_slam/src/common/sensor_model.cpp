#include "common/sensor_model.hpp"
#include "common/utils.hpp"

#include <cmath>

SensorModel::SensorModel()
    : ray_stride_{RAY_STRIDE}
{}

double SensorModel::occupancyToLogOdds(int8_t occupancy) const
{
    if (occupancy < 0) return 0.0;  // Unknown cell
    double p = std::clamp(occupancy / 100.0, 0.01, 0.99);
    return std::log(p / (1.0 - p));
}

bool SensorModel::isCellInGrid(int x, int y, const nav_msgs::msg::OccupancyGrid& map) const
{
    return (x >= 0 && x < static_cast<int>(map.info.width) &&
            y >= 0 && y < static_cast<int>(map.info.height));
}

int8_t SensorModel::getOccupancy(int x, int y, const nav_msgs::msg::OccupancyGrid& map) const
{
    if (!isCellInGrid(x, y, map)) return -1;
    return map.data[y * map.info.width + x];
}

bool SensorModel::worldToGrid(double world_x, double world_y,
                              const nav_msgs::msg::OccupancyGrid& map,
                              int& grid_x, int& grid_y) const
{
    const auto& origin = map.info.origin;
    const float res = map.info.resolution;
    grid_x = static_cast<int>(std::floor((world_x - origin.position.x) / res));
    grid_y = static_cast<int>(std::floor((world_y - origin.position.y) / res));
    return isCellInGrid(grid_x, grid_y, map);
}

double SensorModel::likelihood(const MovingLaserScan& scan,
                               const nav_msgs::msg::OccupancyGrid& map) const
{
    double score = 0.0;
    int ray_count = 0;
    const float res = map.info.resolution;

    for (const auto& ray : scan) {
        if ((ray_count++ % ray_stride_) != 0) continue;

        // Lidar is mounted backward (180 degree offset)
        double angle = wrapToPi(ray.theta + M_PI);
        double dx = std::cos(angle);
        double dy = std::sin(angle);

        // Ray endpoint
        double end_x = ray.origin.x + ray.range * dx;
        double end_y = ray.origin.y + ray.range * dy;

        int gx, gy;
        if (!worldToGrid(end_x, end_y, map, gx, gy)) continue;

        // Check endpoint cell
        double log_odds = occupancyToLogOdds(getOccupancy(gx, gy, map));
        if (log_odds > 0) { 
            // TODO: The endpoint is a hit, how do we score the ray?
            // score = ?
        }
        // TODO: if endpoint cell not a hit - check one cell after (along ray)
        // TODO: if cell after not a hit - check one cell before (along ray)
        // Hint if we check before/after cell, we take a fraction
        // you can use ADJACENT_FRACTION in sensor_model.hpp
        // you can also tune RAY_STRIDE to skip ray for faster computing
        (void)res;
    }

    return score;
}

nav2_msgs::msg::ParticleCloud SensorModel::weightParticles(const nav2_msgs::msg::ParticleCloud& proposal,
                                                           const nav2_msgs::msg::ParticleCloud& parent_cloud,
                                                           const sensor_msgs::msg::LaserScan&   scan,
                                                           const nav_msgs::msg::OccupancyGrid&  map) const
{
    nav2_msgs::msg::ParticleCloud posterior = proposal;
    double sum_w = 0.0;

    for (size_t i = 0; i < posterior.particles.size(); ++i) {
        MovingLaserScan deskewed_scan(scan, parent_cloud.particles[i].pose, proposal.particles[i].pose);
        posterior.particles[i].weight = likelihood(deskewed_scan, map);
        sum_w += posterior.particles[i].weight;
    }

    // Avoid division by zero
    if (sum_w <= 0.0) {
        const double w = 1.0 / static_cast<double>(posterior.particles.size());
        for (auto& p : posterior.particles) p.weight = w;
        return posterior;
    }

    // Normalize
    for (auto& p : posterior.particles) p.weight /= sum_w;

    return posterior;
}
