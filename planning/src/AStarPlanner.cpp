#include <ros/ros.h>
#include <set>
#include <vector>

#include "planning/AStarPlanner.h"

AStarPlanner::AStarPlanner() {
    std::pair<int, int> VALID_SEARCH_DIRECTIONS[NUM_SEARCH_DIRECTIONS] = {
        std::make_pair(-1,0),  // left
        std::make_pair(1,0),   // right
        std::make_pair(0,-1),  // up
        std::make_pair(0,1),   // down
        std::make_pair(-1,1),
        std::make_pair(-1,-1),
        std::make_pair(1,1),
        std::make_pair(1,-1)
    };
}

RobotPath AStarPlanner::planPath(const int **map,
    const int &map_w,
    const int &map_h,
    const std::pair<int, int> &start_pos,
    const std::pair<int, int> &end_pos) {

    // Initialize open and closed set
    std::set<PlannerCell*, PlannerCellPtrComp> open_set; // Cells to be explored, sorted by lowest cost
    std::set<std::pair<int, int> > closed_set; // Indices of cells that have already been explored
    // The closed set could technically be an unordered_set for improved runtime (but then we need to write our own hash function)

    // Initialize open set with start position indices
    PlannerCell *start_cell = new PlannerCell(start_pos);
    open_set.insert(start_cell);

    // Keep track of path
    RobotPath path;

    // Run A* algorithm
    while (!open_set.empty()) {
        // Get lowest cost cell from the open set
        PlannerCell* curr_cell = *(open_set.begin());

        if (curr_cell->indices == end_pos) {
            // Goal has been found
            // Reconstruct path
            while (curr_cell) {
                path.addToFront(curr_cell->indices);
                curr_cell = curr_cell->parent;
            }
            // Deallocate memory
            for (std::set<PlannerCell*>::iterator it = open_set.begin(); it != open_set.end(); ++it) {
                free(*it);
            }
            return path;
        }

        // Goal has not been found
        // Loop through valid adjacent cells
        for (int i = 0; i < NUM_SEARCH_DIRECTIONS; ++i) {
            const std::pair<int, int>& direction_pair = VALID_SEARCH_DIRECTIONS[i];
            const std::pair<int, int>& adj_cell_indices = std::make_pair(curr_cell->indices.first + direction_pair.first,
                curr_cell->indices.second + direction_pair.second);

            // Ignore cells that have already been explored, or cells that are obstacles/have invalid indices
            if (closed_set.count(adj_cell_indices) || isObstacle(adj_cell_indices, map, map_w, map_h)) {
                continue;
            }
            PlannerCell *adj_cell = new PlannerCell(adj_cell_indices);
            adj_cell->parent = curr_cell;

            // Calculate cost for cell
            // g_cost is the distance required to travel from start to adj_cell, multiplied by a cost factor
            adj_cell->g_cost = adj_cell->parent->g_cost + sqrt((direction_pair.first * direction_pair.first
                + direction_pair.second * direction_pair.second) * getCost(adj_cell_indices, map, map_w, map_h) + 1);
            // h_cost is defined by the cost heuristic function
            adj_cell->h_cost = costHeuristic(adj_cell_indices, end_pos);

            // Add cell to open set
            open_set.insert(adj_cell);
        }

        // Remove current cell from open set and place in closed set
        closed_set.insert(curr_cell->indices);
        open_set.erase(curr_cell);
    }

    ROS_ERROR("Unable to find path with A*!");
    // Deallocate memory
    for (std::set<PlannerCell*>::iterator it = open_set.begin(); it != open_set.end(); ++it) {
        free(*it);
    }        
    // Return an empty path
    return path;
}

float AStarPlanner::costHeuristic(const std::pair<int, int> &start_pos,
    const std::pair<int, int> &end_pos) const {
    return sqrt((start_pos.first - end_pos.first) * (start_pos.first - end_pos.first) +
        (start_pos.second - end_pos.second) * (start_pos.second - end_pos.second));
}

bool AStarPlanner::isValidIndices(std::pair<int, int> indices,
    const int &map_w,
    const int &map_h) const {
    int i = indices.first, j = indices.second;
    return i >= 0 && i < map_h && j >= 0 && j < map_w;
}

bool AStarPlanner::isObstacle(std::pair<int, int> indices,
    const int **map,
    const int &map_w,
    const int &map_h) const {
    if (!isValidIndices(indices, map_w, map_h)) {
        return true;
    }
    int i = indices.first, j = indices.second;
    return map[i][j] >= OBSTACLE_COST;
}

int AStarPlanner::getCost(std::pair<int, int> indices,
    const int **map,
    const int &map_w,
    const int &map_h) const {
    if (!isValidIndices(indices, map_w, map_h)) {
        return OBSTACLE_COST;
    }
    int i = indices.first, j = indices.second;
    return map[i][j];
}