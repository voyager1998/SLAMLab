#include <limits>
#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <queue>

class Compare {
public:
    bool operator()(node n1, node n2) {  // n1<n2 for find max
        return n1.fscore > n2.fscore;
    }
};

robot_path_t reconstruct_path(std::vector<coordinate> camefrom, node curr_node) {
    robot_path_t path;
    return path;
}

robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params) {
    ////////////////// TODO: Implement your A* search here //////////////////////////
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);

    // std::vector<coordinate> openset;
    // openset.push_back(distances.cellIndexLookup(start));

    std::vector<coordinate> camefrom;

    std::vector<std::vector<float>> gScore(distances.heightInCells(), std::vector<float>(distances.widthInCells(), std::numeric_limits<float>::infinity()));
    gScore[distances.cellIndexLookup(start).x][distances.cellIndexLookup(start).y] = 0;

    std::vector<std::vector<float>> fScore(distances.heightInCells(), std::vector<float>(distances.widthInCells(), std::numeric_limits<float>::infinity()));
    fScore[distances.cellIndexLookup(start).x][distances.cellIndexLookup(start).y] = heuristic(goal, start);

    std::priority_queue<node, std::vector<node>, Compare> openset;
    node firstnode;
    firstnode.coor = distances.cellIndexLookup(start);
    firstnode.fscore = heuristic(goal, start);
    openset.push(firstnode);

    coordinate goal_coor = distances.cellIndexLookup(goal);

    while (!openset.empty()) {
        node curr_node = openset.top();
        if (curr_node.coor.x == goal_coor.x && curr_node.coor.y == goal_coor.y) {
            return reconstruct_path(camefrom, curr_node);
        }
    }

    path.path_length = path.path.size();
    return path;
}

float heuristic(pose_xyt_t goal, pose_xyt_t p) {
    float euclideanDist = sqrt((goal.x - p.x) * (goal.x - p.x) + (goal.y - p.y) * (goal.y - p.y));
    return euclideanDist;
}
