#ifndef A_STAR_PLANNER
#define A_STAR_PLANNER

#include <ros/ros.h>

#include "planning/RobotPath.h"

struct PlannerCell {
    PlannerCell(const std::pair<int, int>& indices) {
        this->indices = indices;
        parent = NULL;
        g_cost = h_cost = 0.0;
    }

    // bool operator<(const PlannerCell& rhs) const {
    //     return cost() < rhs.cost();
    // }

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

struct PlannerCellPtrComp {
    bool operator()(const PlannerCell* lhs, const PlannerCell* rhs) const {
        if (lhs->cost() == rhs->cost()) return lhs->indices < rhs->indices;
        return lhs->cost() < rhs->cost();
    }
};

class AStarPlanner {
public:
    AStarPlanner();

    // Instead of passing a map 2D array, width and height, ideally there
    // would be a Map data structure that gets passed in
    RobotPath planPath(int **map,
        const int &map_w,
        const int &map_h,
        const std::pair<int, int> &start_pos,
        const std::pair<int, int> &end_pos);

private:
    static const int NUM_SEARCH_DIRECTIONS = 8;
    int VALID_SEARCH_DIRECTIONS[NUM_SEARCH_DIRECTIONS][2];

    float costHeuristic(const std::pair<int, int> &start_pos,
        const std::pair<int, int> &end_pos) const;

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