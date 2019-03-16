#ifndef A_STAR_PLANNER
#define A_STAR_PLANNER

#include <ros/ros.h>

#include "planning/RobotPath.h"

// A struct representing one map cell, for planning purposes
struct PlannerCell {
    PlannerCell(const std::pair<int, int>& indices) {
        this->indices = indices;
        parent = NULL;
        g_cost = h_cost = 0.0;
    }

    void print() const {
        ROS_INFO("PlannerCell @ {%d, %d}: g,h costs: {%f, %f}",
            indices.first, indices.second, g_cost, h_cost);
    }

    float cost() const {
        return g_cost + h_cost;
    }

    PlannerCell *parent; // parent cell in path search
    std::pair<int, int> indices; // row, col indices in costmap
    float g_cost; // Cost to travel from start to PlannerCell
    float h_cost; // Heuristic cost (estimated cost to travel from PlannerCell to end)
};

// Comparator struct to order PlannerCells by cost
struct PlannerCellPtrComp {
    bool operator()(const PlannerCell* lhs, const PlannerCell* rhs) const {
        if (lhs->cost() == rhs->cost()) return lhs->indices < rhs->indices;
        return lhs->cost() < rhs->cost();
    }
};

// Path-Planner using A* Algorithm
class AStarPlanner {
public:
    AStarPlanner();

    /**
     * @brief Generates a path using A* algorithm
     *        A decent explanation can be found here: https://www.geeksforgeeks.org/a-search-algorithm/
     * @param map: Costmap representation (TODO: Change this to a Map data structure when it becomes available)
     * @param map_w, map_h: Width and height of map (number of cols and rows in the array)
     * @param start_pos, end_pos: Start and goal points, in (row,col) format
     */
    RobotPath planPath(int **map,
        const int &map_w,
        const int &map_h,
        const std::pair<int, int> &start_pos,
        const std::pair<int, int> &end_pos);

private:
    static const int NUM_SEARCH_DIRECTIONS = 4;
    int VALID_SEARCH_DIRECTIONS[NUM_SEARCH_DIRECTIONS][2]; // This gets initialized in constructor
    static const float TURN_COST = 10; // Cost for turning, experimentally determined

    /**
     * @brief Calculate heuristic (h-cost) based on Euclidean distance to goal
     * @param start_pos, end_pos: Current position and goal position, in (row, col) format 
     * @param is_turn: Whether the robot requires turning to get to this next cell
     */
    float costHeuristic(const std::pair<int, int> &start_pos,
        const std::pair<int, int> &end_pos,
        const bool &is_turn) const;

    // These functions and constants would ideally be part of some Map data structure later
    static const int OBSTACLE_COST = 100;
    bool isValidIndices(std::pair<int, int> indices,
        const int &map_w,
        const int &map_h) const;
    bool isObstacle(std::pair<int, int> indices,
        int **map,
        const int &map_w,
        const int &map_h) const;
    int getCost(std::pair<int, int> indices,
        int **map,
        const int &map_w,
        const int &map_h) const;
};

#endif  // A_STAR_PLANNER