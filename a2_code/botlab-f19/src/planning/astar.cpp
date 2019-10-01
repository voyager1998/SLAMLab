#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <map>
using namespace std;


robot_path_t reconstruct_path(map<pose_xyt_t, pose_xyt_t> cameFrom, pose_xyt_t current)
{
    robot_path_t total_path;
    total_path.utime = current.utime;
    total_path.path.push_back(current);
    total_path.path_length = total_path.path.size();
    while (cameFrom.find(current) != cameFrom.end())
    {
        current = cameFrom[current];
        total_path.utime = current.utime;
        total_path.path.insert(total_path.path.begin(), current);
        total_path.path_length = total_path.path.size();
    }
    return total_path;
}


float heuristic(pose_xyt_t a, pose_xyt_t b)
{
    float euclideanDist = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    return euclideanDist;
} 


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();

    map<pose_xyt_t, pose_xyt_t> cameFrom;
    return path;
}
