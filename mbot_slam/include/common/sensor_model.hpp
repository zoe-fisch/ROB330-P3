#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include "common/moving_laser_scan.hpp"

#include <cmath>

class SensorModel
{
public:
    static constexpr int RAY_STRIDE = 2;
    static constexpr double ADJACENT_FRACTION = 0.5;

    explicit SensorModel();

    double likelihood(const MovingLaserScan& scan,
                      const nav_msgs::msg::OccupancyGrid& map) const;

    // Apply sensor model to a proposal cloud: weight each particle by scan likelihood.
    // parent_cloud is the cloud before propagation, used for scan interpolation.
    nav2_msgs::msg::ParticleCloud weightParticles(const nav2_msgs::msg::ParticleCloud& proposal,
                                                  const nav2_msgs::msg::ParticleCloud& parent_cloud,
                                                  const sensor_msgs::msg::LaserScan&   scan,
                                                  const nav_msgs::msg::OccupancyGrid&  map) const;

private:
    // occupancy is probability percentage [0-100]
    double occupancyToLogOdds(int8_t occupancy) const;

    bool worldToGrid(double world_x, double world_y,
                     const nav_msgs::msg::OccupancyGrid& map,
                     int& grid_x, int& grid_y) const;

    bool isCellInGrid(int x, int y, const nav_msgs::msg::OccupancyGrid& map) const;
    int8_t getOccupancy(int x, int y, const nav_msgs::msg::OccupancyGrid& map) const;

    int ray_stride_;
};
