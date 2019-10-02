#include <common/grid_utils.hpp>
#include <lcmtypes/particle_t.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/sensor_model.hpp>

#define LIDARCORRECT -4
#define LIDARLONGER -12
#define LIDARSHORTER -8

SensorModel::SensorModel(void) {
    ///////// TODO: Handle any initialization needed for your sensor model
}

double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map) {
    // Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    double logP = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    for(auto adj_ray_iter = movingScan.begin(); adj_ray_iter < movingScan.end(); adj_ray_iter++)
    {
        auto ray = *adj_ray_iter;
        // ray.range += 0.07;
        // coordinate end_pt = coordinate_convert_.get_end_point_coordinate(ray, map);
        ray_coordinates ray_pts = coordinate_convert_.get_ray_coordinates(ray, map);

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
        if(findwall==false){//lidar is shorter
            logP += LIDARSHORTER;
        }
    }

    return logP;
}
