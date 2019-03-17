#include <ros/ros.h>

#include "mapperception/Map.h"
#include "mapperception/MapRow.h"
#include "planning/AStarPlanner.h"
#include "planning/ControlCommand.h"
#include "planning/RobotPath.h"
#include "planning/GoalPlanner.h"

const float ANGLE_TOL_RAD = 1e-3; // Tolerance used for floating point equality comparisons

// TODO Check what happens if -1 is returned, e.g. while we are turning
float robotDirectionToAngle(const RobotDirections &direction) {
    switch (direction) {
        case RobotDirections::UP: return 0.5*M_PI;
        case RobotDirections::DOWN: return 1.5*M_PI;
        case RobotDirections::LEFT return M_PI;
        case RobotDirections::RIGHT return 0;
        default: return -1;
    }
}

// TODO Check what happens if -1 is returned, e.g. while we are turning
// This is basically a copy of the GoalPlanner function; would be better if it was only implemented in one place
float getAngleToNextCell(const std::pair<int, int> &curr_indices,
    const std::pair<int, int> &goal_indices) const {
    int i_diff = goal_indices.first - curr_indices.first;
    int j_diff = goal_indices.second - curr_indices.second;

    if (i_diff == 1 && j_diff == 0) return 1.5*M_PI;
    else if (i_diff == -1 && j_diff == 0) return 0.5*M_PI;
    else if (i_diff == 0 && j_diff == 1) return 0;
    else if (i_diff == 0 && j_diff == -1) return M_PI;
        
    return -1;
}

// Returns proper value of direction_is_right
// thetas from 0 to 2 pi
bool getTurningDirection(const float &robot_theta, const float &destination_theta) {
    if (destination_theta > robot_theta) {
        return destination_theta - robot_theta > M_PI;
    } else {
        return robot_theta - destination_theta < M_PI;
    }
}

planning::ControlCommand pathToControlCommand(const RobotPath &path, const float &robot_theta, const GoalPose &goal) {
    planning::ControlCommand control_cmd;
    if (path.size() == 0) {
        // No A* solution was found
        control_cmd.command_type = planning::ControlCommand::STOP;
        control_cmd.direction_is_right = true; // Doesn't matter
        control_cmd.speed_r = control_cmd.speed_l = 0;
        return control_cmd;
    }

    if (path[0].first == goal.i && path[0].second == goal.j) {
        // We are at the right position
        float goal_angle = robotDirectionToAngle(goal.direction);
        if (goal.direction == RobotDirections::ANY || fabs(goal_angle - robot_theta) < ANGLE_TOL_RAD) {
            // We are at the goal (right position and orientation)
            control_cmd.command_type = planning::ControlCommand::STOP;
            control_cmd.direction_is_right = true;
            control_cmd.speed_r = control_cmd.speed_l = 0;
        } else {
            // We need to turn on the spot (right position, wrong orientation)
            control_cmd.command_type = planning::ControlCommand::TURN_ON_SPOT;
            control_cmd.direction_is_right = getTurningDirection(robot_theta, goal_angle);
            control_cmd.speed_r = control_cmd.speed_l = 90; // TODO test a good speed (do we still need this parameter?)
        }
    } else {
        // We are not at the right position
        float destination_angle = getAngleToNextCell(path[0], path[1]);
        if (fabs(destination_angle - robot_theta) < TOL) {
            // Continue going straight
            control_cmd.command_type = planning::ControlCommand::STRAIGHT_LINE;
            control_cmd.direction_is_right = true; // Forward
        } else {
            // Turn on the spot
            control_cmd.command_type = planning::ControlCommand::TURN_ON_SPOT;
            control_cmd.direction_is_right = getTurningDirection(robot_theta, destination_angle);
        }
        control_cmd.speed_r = control_cmd.speed_l = 90; // TODO test a good speed (do we still need this parameter?)
    }

    return control_cmd;
}

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