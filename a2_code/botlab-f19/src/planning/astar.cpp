#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <vector>
#include <map>
#include <queue>
#include <math.h>
#include <limits>
#include <iostream>
#include <time.h>
#include <stdlib.h>
using namespace std;


struct node
{
    Point<int> coor;
    float fs;
    bool operator<(const node& other) const {
        return fs < other.fs;
    }
    bool operator==(const node& other) const {
        return coor.x == other.coor.x && coor.y == other.coor.y; 
    }
};


struct cmp
{
    bool operator()(const node& a, const node& b) const
    {
        return a.fs > b.fs ;
    }
};


robot_path_t reconstruct_path(map<Point<int>, Point<int>> cameFrom, Point<int> current, const ObstacleDistanceGrid& distances, pose_xyt_t start, pose_xyt_t goal)
{
    robot_path_t total_path;
    total_path.path.push_back(goal);
    total_path.path_length = total_path.path.size();
    while (cameFrom.find(current) != cameFrom.end())
    {
        current = cameFrom[current];
        total_path.path.insert(total_path.path.begin(), distances.coorTopose(current));
        total_path.path_length = total_path.path.size();
    }
    // total_path.path[0].theta = 0.0f;
    // for (int i = 0; i < total_path.path_length - 1; i++)
    // {
    //     float dx = total_path.path[i + 1].x - total_path.path[i].x;
    //     float dy = total_path.path[i + 1].y - total_path.path[i].y;
    //     if (dx == 0 && dy < 0) total_path.path[i].theta = M_PI / 2;
    //     if (dx == 0 && dy > 0) total_path.path[i].theta = 3 * M_PI / 2;
    //     if (dy == 0 && dx < 0) total_path.path[i].theta = M_PI;
    //     if (dy == 0 && dx > 0) total_path.path[i].theta = 0;
    //     if (dx < 0 && dy < 0) total_path.path[i].theta = M_PI - atan2(-dy, -dx);
    //     if (dx < 0 && dy > 0) total_path.path[i].theta = M_PI + atan2(dy, -dx);
    //     if (dx > 0 && dy < 0) total_path.path[i].theta = atan2(-dy, dx);
    //     if (dx > 0 && dy > 0) total_path.path[i].theta = 2 * M_PI - atan2(dy, dx);
    // }
    // total_path.path[total_path.path_length - 1].theta = 0.0f;
    total_path.path[0] = start;
    return total_path;
}


float heuristic(Point<int> a, Point<int> b)
{
    srand(time(NULL));
    // float euclideanDist = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    // return euclideanDist;
    float manhattanDist = abs(a.x - b.x) + abs(a.y - b.y); // +((double)rand() / (RAND_MAX));
    return manhattanDist;
}


bool inContainer(vector<Point<int>> ctn, Point<int> item)
{
    bool res = false;
    for (auto i : ctn) {
        if (i == item) res = true;
    }
    return res;
}


vector<int> nx = {1, -1, 0, 0, 1, 1, -1, -1};
vector<int> ny = {0, 0, 1, -1, 1, -1, 1, -1};


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    // std::cout <<  "#################################" << std::endl;
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();

    Point<int> startGrid = distances.poseToCoor(start);
    Point<int> goalGrid = distances.poseToCoor(goal);
    // cout << "*******" << endl;
    // cout << startGrid.x << ' ' << startGrid.y << endl;
    // cout << goalGrid.x << ' ' << goalGrid.y << endl;
    priority_queue<node, vector<node>, cmp> openSet;
    vector<Point<int>> inOpen;
    vector<Point<int>> closedSet;
    map<Point<int>, Point<int>> cameFrom;
    map<Point<int>, float> gScore;
    gScore[startGrid] = 0;
    map<Point<int>, float> fScore;
    fScore[startGrid] = heuristic(goalGrid, startGrid);
    node pos;
    pos.coor = startGrid;
    pos.fs = fScore[startGrid];
    openSet.push(pos);
    inOpen.push_back(startGrid);
    while (!openSet.empty())
    {
        Point<int> current = openSet.top().coor;
        if (current == goalGrid)
        {
            return reconstruct_path(cameFrom, current, distances, start, goal);
        }
        closedSet.push_back(current);
        openSet.pop();
        size_t idx = 0;
        for (size_t i = 0; i < inOpen.size(); ++i) {
            if (inOpen[i] == current) {
                idx = i;
                break;
            }
        }
        inOpen.erase(inOpen.begin() + idx);
        vector<Point<int>> neighbors;
        for (size_t i = 0; i < 8; i++)
        {
            Point<int> neighbor;
            neighbor.x = current.x + nx[i];
            neighbor.y = current.y + ny[i];
            if (!distances.isCellInGrid(neighbor.x, neighbor.y)) continue;
            if (distances(neighbor.x, neighbor.y) > params.minDistanceToObstacle + 0.2)
            {
                neighbors.push_back(neighbor);
            }
        }       
        for (auto neighbor : neighbors)
        {
            if (inContainer(closedSet, neighbor))
            {
                continue;
            }
            if (gScore.find(current) == gScore.end()) 
            {
                gScore[current] = numeric_limits<float>::infinity();
                // gScore[current] = 10000.0f;
            }
            float tentative_gScore = gScore[current] + distance_between_points(neighbor, current);
            if (gScore.find(neighbor) == gScore.end()) 
            {
                gScore[neighbor] = numeric_limits<float>::infinity();
                // gScore[neighbor] = 10000.0f;
            }
            if (tentative_gScore < gScore[neighbor])
            {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;
                fScore[neighbor] = gScore[neighbor] + heuristic(goalGrid, neighbor);
                if (!inContainer(inOpen, neighbor))
                {
                    node pos;
                    pos.coor = neighbor;
                    pos.fs = fScore[neighbor];
                    openSet.push(pos);
                    inOpen.push_back(neighbor);
                }
                
            }   
        }
    }
    
    return path;
}
