#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


coordinate ObstacleDistanceGrid::poseToCoor(pose_xyt_t pos) const
{
    coordinate coor;
    coor.x = (int)floor((pos.x - globalOrigin_.x) / metersPerCell_);
    coor.y = (int)floor((pos.y - globalOrigin_.y) / metersPerCell_);
    return coor;
}


pose_xyt_t ObstacleDistanceGrid::coorTopose(coordinate current) const
{
    pose_xyt_t pos;
    // coor.x = (int)floor((pos.x - globalOrigin_.x) / metersPerCell_);
    // coor.y = (int)floor((pos.y - globalOrigin_.y) / metersPerCell_);
    pos.x = (float)(current.x * metersPerCell_ + globalOrigin_.x);
    pos.y = (float)(current.y * metersPerCell_ + globalOrigin_.y);
    return pos;
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
