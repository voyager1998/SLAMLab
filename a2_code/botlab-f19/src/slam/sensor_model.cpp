#include <common/grid_utils.hpp>
#include <lcmtypes/particle_t.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/sensor_model.hpp>

#define LIDARCORRECT -4
#define LIDARLONGER -12
#define LIDARSHORTER -8

#define SIGMA 2
#define RANGE 0.3

SensorModel::SensorModel(void) {
    // Handle any initialization needed for your sensor model
}

double SensorModel::likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map) {
    // Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    double logP = 0.0;
    MovingLaserScan movingScan(scan, particle.parent_pose, particle.pose);
    for(auto adj_ray_iter = movingScan.begin(); adj_ray_iter < movingScan.end(); adj_ray_iter++)
    {
        auto ray = *adj_ray_iter;
        // ray.range += 0.07;
        coordinate end_pt = coordinate_convert_.get_end_point_coordinate(ray, map);
        ray_coordinates ray_pts = coordinate_convert_.get_ray_coordinates(ray, map);
        ray_pts.push_back(end_pt);

        int num_cells = ray_pts.size();
        bool findwall = false;
        for (int i = 0; i < num_cells; i++) {
            coordinate temp = ray_pts[i];
            if (map(temp.x,temp.y) > 0){
                findwall = true;
                if (num_cells - i > 1) {  // lidar is longer
                    logP += LIDARLONGER;
                    break;
                } else {// lidar is just in place
                    logP += LIDARCORRECT;
                    break;
                }
            }
        }
        if(!findwall){//lidar is shorter
            logP += LIDARSHORTER;
        }
    }

    return logP;
}

double SensorModel::Gaussianlikelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map) {
    double logP = 0.0;
    MovingLaserScan movingScan(scan, particle.parent_pose, particle.pose);
    for (auto adj_ray_iter = movingScan.begin(); adj_ray_iter < movingScan.end(); adj_ray_iter++) {
        auto ray = *adj_ray_iter;
        ray.range += RANGE;
        coordinate end_pt = coordinate_convert_.get_end_point_coordinate(ray, map);
        ray_coordinates ray_pts = coordinate_convert_.get_ray_coordinates(ray, map);

        int num_cells = ray_pts.size();
        bool findwall = false;
        for (int i = 0; i < num_cells; i++) {
            coordinate temp = ray_pts[i];
            if (map(temp.x, temp.y) > 0) {
                findwall = true;
                // when dist == SIGMA, logP += -0.5
                double distance_to_endpt = dist(temp, end_pt, map.metersPerCell());
                // if (distance_to_endpt > 2 * SIGMA) std::cout << "warining: dist > map.height" << std::endl;
                logP += -pow(distance_to_endpt, 2) / (2 * pow(SIGMA, 2));
                break;
            }
        }
        if (findwall == false) {  //lidar is shorter
            logP += -pow(RANGE, 2) / (2 * pow(SIGMA, 2)) - 0.5;
        }
    }

    return logP;
}