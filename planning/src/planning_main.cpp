#include <ros/ros.h>

#include "mapperception/Map.h"
#include "mapperception/MapRow.h"
#include "planning/AStarPlanner.h"
#include "planning/Arc.h"
#include "planning/RobotPath.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;

    // Later replace this with subscribers
    const int map_w = 20, map_h = 20;
    int **map = new int *[map_h];
    for (int i = 0; i < map_h; ++i) {
        map[i] = new int[map_w];
        for (int j = 0; j < map_w; ++j) {
            map[i][j] = 0;
        }
    }
    // Set some obstacles/walls
    for (int i = 0; i < 3*map_w/4; ++i) {
        map[map_h/2][i] = 100;
        map[map_h/2+1][i] = 100;
    }

    std::pair<int, int> start_pos = std::make_pair(0,0);
    std::pair<int, int> end_pos = std::make_pair(map_h-1, 0);

    // Initialize planner
    AStarPlanner planner;
    RobotPath path = planner.planPath(map, map_w, map_h, start_pos, end_pos);

    path.print(map, map_w, map_h);
    free(map);
    while (ros::ok()) {
        
    }
    return 0;
}