#include <ros/ros.h>
#include <set>
#include <vector>

#include <unistd.h>

#include "planning/AStarPlanner.h"

AStarPlanner::AStarPlanner() {
    // Initialize VALID_SEARCH_DIRECTIONS
    // Could not figure out a smarter way to do this with C++03 :(
    VALID_SEARCH_DIRECTIONS[0][0] = -1; VALID_SEARCH_DIRECTIONS[0][1] = 0;  // up
    VALID_SEARCH_DIRECTIONS[1][0] = 1; VALID_SEARCH_DIRECTIONS[1][1] = 0;   // down
    VALID_SEARCH_DIRECTIONS[2][0] = 0; VALID_SEARCH_DIRECTIONS[2][1] = -1;  // left
    VALID_SEARCH_DIRECTIONS[3][0] = 0; VALID_SEARCH_DIRECTIONS[3][1] = 1;   // right
}

RobotPath AStarPlanner::planPath(
    const mapperception::Map &map,
    const std::pair<int, int> &start_pos,
    const std::pair<int, int> &end_pos) {
    int map_w = 6;
    int map_h = 6;

    // Initialize open and closed set
    std::set<PlannerCell*, PlannerCellPtrComp> open_set; // Cells to be explored, sorted by lowest cost
    std::set<std::pair<int, int> > closed_set; // Indices of cells that have already been explored or queued for exploration
    // The closed set could technically be an unordered_set for improved runtime (but then we need to write our own hash function)

    // Initialize open set with start position indices
    PlannerCell *start_cell = new PlannerCell(start_pos);
    open_set.insert(start_cell);
    closed_set.insert(start_cell->indices);

    // Keep track of path
    RobotPath path;

    // Run A* algorithm
    // A decent explanation can be found here https://www.geeksforgeeks.org/a-search-algorithm/
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
            const std::pair<int, int>& direction_pair = std::make_pair(VALID_SEARCH_DIRECTIONS[i][0], VALID_SEARCH_DIRECTIONS[i][1]);
            const std::pair<int, int>& adj_cell_indices = std::make_pair(curr_cell->indices.first + direction_pair.first,
                curr_cell->indices.second + direction_pair.second);
            // Ignore cells that have already been explored, or cells that are obstacles/have invalid indices
            if (closed_set.count(adj_cell_indices) || isObstacle(adj_cell_indices, map, map_w, map_h)) {
                continue;
            }
            closed_set.insert(adj_cell_indices);
            
            PlannerCell *adj_cell = new PlannerCell(adj_cell_indices);
            adj_cell->parent = curr_cell;

            // Calculate cost for cell
            // g_cost is the distance required to travel from start to adj_cell, multiplied by a cost factor
            adj_cell->g_cost = adj_cell->parent->g_cost + sqrt((direction_pair.first * direction_pair.first
                + direction_pair.second * direction_pair.second) * (getCost(adj_cell_indices, map, map_w, map_h) + 1));
            // h_cost is defined by the cost heuristic function
            bool is_turn = false;
            if (curr_cell->parent) {
                std::pair<int, int> parent_indices = curr_cell->parent->indices;
                int x_change = parent_indices.first - curr_cell->indices.first;
                int y_change = parent_indices.second - curr_cell->indices.second;
                is_turn = !(x_change == curr_cell->indices.first - adj_cell_indices.first && y_change == curr_cell->indices.second - adj_cell_indices.second); 
            }
            adj_cell->h_cost = costHeuristic(adj_cell_indices, end_pos, is_turn);
            // Add cell to open set
            open_set.insert(adj_cell);
        }

        // Remove current cell from open set
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
    const std::pair<int, int> &end_pos,
    const bool &is_turn) const {
    float cost = sqrt((start_pos.first - end_pos.first) * (start_pos.first - end_pos.first) +
        (start_pos.second - end_pos.second) * (start_pos.second - end_pos.second));
    // Punish turns
    if (is_turn) {
        cost += TURN_COST;
    }
    return cost;
}

bool AStarPlanner::isValidIndices(std::pair<int, int> indices,
    const int &map_w,
    const int &map_h) const {
    int i = indices.first, j = indices.second;
    return i >= 0 && i < map_h && j >= 0 && j < map_w;
}

bool AStarPlanner::isObstacle(std::pair<int, int> indices,
    const mapperception::Map &map,
    const int &map_w,
    const int &map_h) const {
    if (!isValidIndices(indices, map_w, map_h)) {
        return true;
    }
    int i = indices.first, j = indices.second;
    return map.map[i].row[j] >= OBSTACLE_COST;
}

int AStarPlanner::getCost(std::pair<int, int> indices,
    const mapperception::Map &map,
    const int &map_w,
    const int &map_h) const {
    if (!isValidIndices(indices, map_w, map_h)) {
        return OBSTACLE_COST;
    }
    int i = indices.first, j = indices.second;
    return map.map[i].row[j];
}