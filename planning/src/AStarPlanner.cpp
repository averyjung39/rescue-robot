#include <ros/ros.h>
#include <set>
#include <vector>

#include <unistd.h>

#include "planning/AStarPlanner.h"

AStarPlanner::AStarPlanner() {
    // Initialize VALID_SEARCH_DIRECTIONS
    // Could not figure out a smarter way to do this with C++03 :(
    VALID_SEARCH_DIRECTIONS[0][0] = -1; VALID_SEARCH_DIRECTIONS[0][1] = 0;  // left
    VALID_SEARCH_DIRECTIONS[1][0] = 1; VALID_SEARCH_DIRECTIONS[1][1] = 0;   // right
    VALID_SEARCH_DIRECTIONS[2][0] = 0; VALID_SEARCH_DIRECTIONS[2][1] = -1;  // up
    VALID_SEARCH_DIRECTIONS[3][0] = 0; VALID_SEARCH_DIRECTIONS[3][1] = 1;   // down
    VALID_SEARCH_DIRECTIONS[4][0] = -1; VALID_SEARCH_DIRECTIONS[4][1] = 1;
    VALID_SEARCH_DIRECTIONS[5][0] = -1; VALID_SEARCH_DIRECTIONS[5][1] = -1;
    VALID_SEARCH_DIRECTIONS[6][0] = 1; VALID_SEARCH_DIRECTIONS[6][1] = 1;
    VALID_SEARCH_DIRECTIONS[7][0] = 1; VALID_SEARCH_DIRECTIONS[7][1] = -1;
}

RobotPath AStarPlanner::planPath(int **map,
    const int &map_w,
    const int &map_h,
    const std::pair<int, int> &start_pos,
    const std::pair<int, int> &end_pos) {

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
                // ROS_WARN("%d", closed_set.count(adj_cell_indices));
                continue;
            }
            closed_set.insert(adj_cell_indices);
            // ROS_WARN("{%d, %d}", adj_cell_indices.first, adj_cell_indices.second);
            
            PlannerCell *adj_cell = new PlannerCell(adj_cell_indices);
            adj_cell->parent = curr_cell;

            // Calculate cost for cell
            // g_cost is the distance required to travel from start to adj_cell, multiplied by a cost factor
            adj_cell->g_cost = adj_cell->parent->g_cost + sqrt((direction_pair.first * direction_pair.first
                + direction_pair.second * direction_pair.second) * (getCost(adj_cell_indices, map, map_w, map_h) + 1));
            // h_cost is defined by the cost heuristic function
            adj_cell->h_cost = costHeuristic(adj_cell_indices, end_pos);

            // Add cell to open set
            // adj_cell->print();
            open_set.insert(adj_cell);
            // ROS_WARN("OPEN SET: ");
            // for (std::set<PlannerCell*>::iterator it = open_set.begin(); it != open_set.end(); ++it) {
            //     (*it)->print();
            // }
            // ROS_WARN("CLOSED SET: ");
            // for (std::set<std::pair<int, int> >::iterator it = closed_set.begin(); it != closed_set.end(); ++it) {
            //     ROS_INFO("PlannerCell @ {%d, %d}", it->first, it->second);
            // }
            // ROS_INFO("\n"); 
        }

        // ROS_WARN("OPEN SET: ");
        // for (std::set<PlannerCell*>::iterator it = open_set.begin(); it != open_set.end(); ++it) {
        //     (*it)->print();
        // }
        // ROS_WARN("CLOSED SET: ");
        // for (std::set<std::pair<int, int> >::iterator it = closed_set.begin(); it != closed_set.end(); ++it) {
        //     ROS_INFO("PlannerCell @ {%d, %d}", it->first, it->second);
        // }

        // Remove current cell from open set and place in closed set
        // closed_set.insert(curr_cell->indices);
        open_set.erase(curr_cell);

        // ROS_WARN("OPEN SET: ");
        // for (std::set<PlannerCell*>::iterator it = open_set.begin(); it != open_set.end(); ++it) {
        //     (*it)->print();
        // }
        // ROS_WARN("CLOSED SET: ");
        // for (std::set<std::pair<int, int> >::iterator it = closed_set.begin(); it != closed_set.end(); ++it) {
        //     ROS_INFO("PlannerCell @ {%d, %d}", it->first, it->second);
        // }
        
        // usleep(500000);
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
    int **map,
    const int &map_w,
    const int &map_h) const {
    if (!isValidIndices(indices, map_w, map_h)) {
        return true;
    }
    int i = indices.first, j = indices.second;
    return map[i][j] >= OBSTACLE_COST;
}

int AStarPlanner::getCost(std::pair<int, int> indices,
    int **map,
    const int &map_w,
    const int &map_h) const {
    if (!isValidIndices(indices, map_w, map_h)) {
        return OBSTACLE_COST;
    }
    int i = indices.first, j = indices.second;
    return map[i][j];
}