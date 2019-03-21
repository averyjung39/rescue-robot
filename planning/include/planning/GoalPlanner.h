#ifndef GOAL_PLANNER
#define GOAL_PLANNER

#include "localization/Pose.h"
#include "mapperception/Map.h"
#include "objectives/ActiveObjectives.h"

enum RobotDirections {
    ROBOT_DIRECTION_UP = 0,
    ROBOT_DIRECTION_DOWN = 1,
    ROBOT_DIRECTION_LEFT = 2,
    ROBOT_DIRECTION_RIGHT = 3,
    ROBOT_DIRECTION_ANY = 4
};

struct GoalPose {
    int i;
    int j;
    RobotDirections direction;
};

class GoalPlanner {
public:
    GoalPlanner();
    GoalPose getGoal(
        const localization::Pose &pose,
        const mapperception::Map &map,
        const objectives::ActiveObjectives &objectives);
private:
    bool _run_initialization;
    std::pair<int, int> _home_indices;

    // Find map indices that has a certain map cell type. Return the one closest to the robot
    bool searchMap(const mapperception::Map &map, const int &label, const int &robot_i, const int &robot_j, std::pair<int, int> &indices) const;

    // Used when we need to set a GoalPose in which we want to face an obstacle
    RobotDirections getDirectionFacingCell(const std::pair<int, int> &curr_indices,
        const std::pair<int, int> &goal_indices) const;

    // Helper function to determine an appropriate goal, e.g. when we want to go to a tile next to an object
    std::pair<int, int> lowestCostIndicesNearby(const mapperception::Map &map, const int &i, const int &j, const int &robot_i, const int &robot_j) const;
    // Determine a goal that will take the robot to the farthest side of the map
    std::pair<int, int> farthestEdgeFromRobot(const int &robot_i, const int &robot_j) const;
    // Determine a goal that will take the robot to the unsearched tile farthest away 
    bool farthestUnsearchedCell(const mapperception::Map &map, const int &robot_i, const int &robot_j, std::pair<int, int> &indices) const;
};

#endif  // GOAL_PLANNER