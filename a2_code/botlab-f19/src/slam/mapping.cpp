#include <slam/mapping.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    if(!initialized_)
    {
        previousPose_ = pose;
    }
    
    MovingLaserScan movingScan(scan, previousPose_, pose);
    
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
}


void Mapping::updateEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    //////////////// TODO: Implement this function //////////////////
}


void Mapping::updateRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    //////////////// TODO: Implement this function //////////////////
}


void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map)
{
    //////////////// TODO: Implement this function //////////////////
}


void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map)
{
    //////////////// TODO: Implement this function //////////////////
}
