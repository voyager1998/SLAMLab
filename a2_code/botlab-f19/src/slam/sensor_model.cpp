#include <common/grid_utils.hpp>
#include <lcmtypes/particle_t.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/sensor_model.hpp>

SensorModel::SensorModel(void) {
    ///////// TODO: Handle any initialization needed for your sensor model
}

double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map) {
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    for(auto adj_ray_iter = movingScan.begin(); adj_ray_iter < movingScan.end(); adj_ray_iter++)
    {
        coordinate end_pt = coordinate_convert_.get_end_point_coordinate(*adj_ray_iter, map);
        ray_coordinates ray_pts = coordinate_convert_.get_ray_coordinates(*adj_ray_iter, map);
    }


    double scanLikelihood = sample.weight;
    return scanLikelihood;
}
