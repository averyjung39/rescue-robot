#ifndef GOAL_PLANNER
#define GOAL_PLANNER

#include "localization/Pose.h"
#include "mapperception/Map.h"
#include "objectives/ActiveObjectives.h"

enum RobotDirections {
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    ANY = 4
};

struct GoalPose {
    int i;
    int j;
    RobotDirections direction;
}

class GoalPlanner {
public:
    GoalPlanner();
    std::pair<int, int> getGoal(
        const localization::Pose &pose,
        const mapperception::Map &map,
        const objectives::ActiveObjectives &objectives);
private:
    bool _run_initialization;
    std::pair<int, int> _home_indices;

    // Find all pair of indices that have a certain map cell type
    std::vector<std::pair<int, int> > queryMap(const mapperception::Map &map, const MapEnum &cell_type) const;

    // Used when we need to set a GoalPose in which we want to face an obstacle
    RobotDirections getDirectionFacingCell(const std::pair<int, int> &curr_indices,
        const std::pair<int, int> &goal_indices) const;
};

#endif  // GOAL_PLANNER