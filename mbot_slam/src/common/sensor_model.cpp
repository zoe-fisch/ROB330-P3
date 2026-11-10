#include "common/sensor_model.hpp"
#include "common/utils.hpp"

#include <algorithm>
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
    const size_t index = static_cast<size_t>(y) * map.info.width + x;
    return index < map.data.size() ? map.data[index] : -1;
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
    if (!(res > 0.0f) || map.data.empty()) return score;

    const auto endpointScore = [&](double x, double y) {
        int gx, gy;
        return worldToGrid(x, y, map, gx, gy)
            ? std::max(0.0, occupancyToLogOdds(getOccupancy(gx, gy, map)))
            : 0.0;
    };

    for (const auto& ray : scan) {
        if ((ray_count++ % ray_stride_) != 0) continue;

        const double dx = std::cos(ray.theta);
        const double dy = std::sin(ray.theta);
        const double end_x = ray.origin.x + ray.range * dx;
        const double end_y = ray.origin.y + ray.range * dy;
        const double hit_score = endpointScore(end_x, end_y);
        if (hit_score > 0.0) {
            score += hit_score;
            continue;
        }

        double adjacent = endpointScore(end_x + res * dx, end_y + res * dy);
        if (adjacent == 0.0 && ray.range > res) {
            adjacent = endpointScore(end_x - res * dx, end_y - res * dy);
        }
        score += ADJACENT_FRACTION * adjacent;
    }

    return score;
}

nav2_msgs::msg::ParticleCloud SensorModel::weightParticles(const nav2_msgs::msg::ParticleCloud& proposal,
                                                           const sensor_msgs::msg::LaserScan&   scan,
                                                           const MovingLaserScan::RelativePoses& scan_poses,
                                                           const nav_msgs::msg::OccupancyGrid&  map) const
{
    nav2_msgs::msg::ParticleCloud posterior = proposal;
    if (posterior.particles.empty() || map.data.empty()) return posterior;

    double sum_w = 0.0;

    for (size_t i = 0; i < posterior.particles.size(); ++i) {
        MovingLaserScan deskewed_scan(scan, scan_poses, posterior.particles[i].pose);
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
