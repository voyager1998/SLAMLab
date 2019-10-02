#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <vector>
#include <map>
#include <queue>
#include <math.h>
#include <limits>
using namespace std;


struct node
{
    Point<int> coor;
    float fs;
};


struct cmp
{
    bool operator()(const node& a, const node& b) const
    {
        return a.fs > b.fs ;
    }
};


robot_path_t reconstruct_path(map<Point<int>, Point<int>> cameFrom, Point<int> current, const ObstacleDistanceGrid& distances)
{
    robot_path_t total_path;
    total_path.path.push_back(distances.coorTopose(current));
    total_path.path_length = total_path.path.size();
    while (cameFrom.find(current) != cameFrom.end())
    {
        current = cameFrom[current];
        total_path.path.insert(total_path.path.begin(), distances.coorTopose(current));
        total_path.path_length = total_path.path.size();
    }
    return total_path;
}


float heuristic(Point<int> a, Point<int> b)
{
    float euclideanDist = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    float manhattanDist = abs(a.x - b.x) + abs(a.y - b.y);
    return manhattanDist;
}


vector<int> nx = {1, -1, 0, 0};
vector<int> ny = {0, 0, 1, -1};


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

    Point<int> startGrid = distances.poseToCoor(start);
    Point<int> goalGrid = distances.poseToCoor(goal);
    priority_queue<node, vector<node>, cmp> openSet;
    vector<Point<int>> inOpen;
    vector<Point<int>> closedSet;
    map<Point<int>, Point<int>> cameFrom;
    map<Point<int>, float> gScore;
    gScore[startGrid] = 0;
    map<Point<int>, float> fScore;
    fScore[startGrid] = heuristic(goalGrid, startGrid);
    node pos;
    pos.coor.x = startGrid.x;
    pos.coor.y = startGrid.y;
    pos.fs = fScore[startGrid];
    openSet.push(pos);
    inOpen.push_back(startGrid);
    while (!openSet.empty())
    {
        Point<int> current = openSet.top().coor;
        if (current.x == goalGrid.x && current.y == goalGrid.y)
        {
            return reconstruct_path(cameFrom, current, distances);
        }
        closedSet.push_back(current);
        openSet.pop();
        remove(inOpen.begin(), inOpen.end(), current);
        vector<Point<int>> neighbors;
        for (size_t i = 0; i < 4; i++)
        {
            Point<int> neighbor;
            neighbor.x = current.x + nx[i];
            neighbor.y = current.y + ny[i];
            // if (distances.isCellInGrid(neighbor.x, neighbor.y))
            // {
            //     neighbors.push_back(neighbor);
            // }
            if ((neighbor.x, neighbor.y) > params.minDistanceToObstacle)
            {
                neighbors.push_back(neighbor);
            }
        }       
        for (auto neighbor : neighbors)
        {
            if (find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end())
            {
                continue;
            }
            float tentative_gScore = gScore[current] + heuristic(neighbor, current);
            if (gScore.find(neighbor) == gScore.end()) 
            {
                gScore[neighbor] = numeric_limits<float>::infinity();
            }
            if (tentative_gScore < gScore[neighbor])
            {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;
                fScore[neighbor] = gScore[neighbor] + heuristic(goalGrid, neighbor);
                if (find(inOpen.begin(), inOpen.end(), neighbor) == inOpen.end())
                {
                    node pos;
                    pos.coor.x = neighbor.x;
                    pos.coor.y = neighbor.y;
                    pos.fs = fScore[neighbor];
                    openSet.push(pos);
                    inOpen.push_back(neighbor);
                }
                
            }   
        }
    }
    
    return path;
}
