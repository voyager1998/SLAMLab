#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>

robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params) {
    ////////////////// TODO: Implement your A* search here //////////////////////////
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);

    std::vector<coordinate> openset;

    openset.push_back(distances.cellIndexLookup(start));
    std::vector<coordinate> camefrom;

    path.path_length = path.path.size();
    return path;
}
