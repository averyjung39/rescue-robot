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
    const int map_w = 6, map_h = 6;
    int **map = new int *[map_h];
    for (int i = 0; i < map_h; ++i) {
        map[i] = new int[map_w];
        for (int j = 0; j < map_w; ++j) {
            map[i][j] = 0;
        }
    }
    // // Set some obstacles/walls
    // map[4][4] = 100;
    // map[4][5] = 100;
    // map[5][4] = 100;
    // map[5][5] = 100;
    // map[6][14] = map[6][15] = map[6][13] = 100;
    // map[7][14] = map[7][15] = map[7][13] = 100;
    // map[8][14] = map[8][15] = map[8][13] = 100;
    // map[10][10] = map[9][10] = 100;
    // map[10][9] = map [9][9] = 100;
    // map[10][11] = map[9][11] = 100;
    // map[16][4] = map[16][3] = map[16][5] = map[16][6] = map[16][6] = 100;
    // map[17][4] = map[17][3] = map[17][5] = map[17][6] = map[17][6] = 100;

    // for (int i = map_w/4; i < map_w; ++i) {
    //     map[map_h/2][i] = 100;
    //     map[map_h/2-1][i] = 100;
    // }
    // for (int i = 0; i < map_w/2; ++i) {
    //     map[2*map_h/3][i] = 100;
    //     map[2*map_h/3+1][i] = 100;
    // }
    // for (int i = map_h/5; i < map_h/2; ++i) {
    //     map[i][map_w/2] = 100;
    // }
    map[4][3] = 100;
    map[4][4] = 100;
    std::pair<int, int> start_pos = std::make_pair(0,2);
    std::pair<int, int> end_pos = std::make_pair(5,5);

    // Initialize planner
    AStarPlanner planner;
    RobotPath path = planner.planPath(map, map_w, map_h, start_pos, end_pos);

    path.print(map, map_w, map_h);
    free(map);
    while (ros::ok()) {
        
    }
    return 0;
}