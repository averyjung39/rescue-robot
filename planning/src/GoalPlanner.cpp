#include "planning/GoalPlanner.h"

GoalPlanner::GoalPlanner() {
    _run_initiaization = true;
    _home_indices = std::make_pair(5,3);
}

GoalPose GoalPlanner::getGoal(
    const localization::Pose &pose,
    const mapperception::Map &map,
    const objectives::ActiveObjectives &objectives) {
    std::vector<bool> objectives_list;

    // TODO: Incorporate timeouts for some of these objectives

    // At the very beginning, sweep left and right
    if (_run_initiaization) {
        if (pose.theta < M_PI) {
            // Keep turning left. Set GoalPose to current indices in map, LEFT
        } else if (pose.theta > 0 && pose.theta < 1.5*M_PI) {
            // Start turning right. Set GoalPose to current indices in map, RIGHT
        } else {
            _run_initiaization = false;
        }
    } else {
        // Start checking objectives
        // Want to find fire first, so check that first
        if (objectives_list[objectives::FIND_FIRE]) {
            // If candle is in the map, set GoalPose to indices adjacent to the candle cell, direction facing the candle
            // Else if there is at least one unknown obstacle in the map, set GoalPose to indices adjacent to closest obstacle, direction facing obstacle
            // Else set goal to some unsearched area
        }
        // If we want to return home, this will be the only active objective, so we should just return home 
        else if (objectives_list[objectives::RETURN_HOME]) {
            // Set goal to home indices, orientation ANY
        } 
        // For all other active objectives, check if any of their locations have already been found in the map
        else if (objectives_list[objectives::FIND_FOOD] && map.contains(magnet)) { // map.contains is pseudocode
            // Set goal to indices of closest magnet, orientation ANY
        } else if (objectives_list[objectives::FIND_SURVIVORS] && map.contains(survivors)) {
            // Set goal to indices adjacent to survivors cell, direction facing survivors
        } else if (objectives_list[objectives::FIND_PERSON] && map.contains(person)) {
            // Set goal to indices adjacent to person cell, direction facing person
        }
        // If we haven't found the locations of these objectives, check unknown obstacles or unsearched areas
        else if (map.contains(unknown_obstacle)) {
            // Set goal to indices adjacent to closest unknown obstacle, direction facing obstacle
        } else if (map.contains(unsearched_space)) {
            // Set goal to indices of closest unsearched space, direction ANY
        } else {
            // Not sure what to do if we get to here. Probably print an error message and either spin on the spot or
            // try re-searching areas

            // Alternatively, we send a signal to objectives saying we weren't able to complete whatever objectives are currently listed
            // as active. Objectives then sends us a new list of objectives to do. (Implementing this would solve the issue of not having timeouts)
        }
    }
}

std::vector<std::pair<int, int> > GoalPlanner::queryMap(const mapperception::Map &map, const MapEnum &cell_type) const {
    // For now, iterate through all map cells
    // If this is found to be too slow, we can create other data structures for fast querying based on cell_type, like a std::unordered_map
    std::vector<std::pair<int, int> > indices_list;
    for (int i = 0; i < map.data.size(); ++i) {
        mapperception::MapRow row = map.data[i];
        for (int j = 0; j < row.data.size(); ++j) {
            if (row.data[i] == cell_type) {
                indices_list.push_back(std::make_pair(i,j));
            }
        }
    }
}

RobotDirections GoalPlanner::getDirectionFacingCell(const std::pair<int, int> &curr_indices,
    const std::pair<int, int> &goal_indices) const {
    int i_diff = goal_indices.first - curr_indices.first;
    int j_diff = goal_indices.second - curr_indices.second;

    if (i_diff == 1 && j_diff == 0) return RobotDirections::DOWN;
    else if (i_diff == -1 && j_diff == 0) return RobotDirections::UP;
    else if (i_diff == 0 && j_diff == 1) return RobotDirections::RIGHT;
    else if (i_diff == 0 && j_diff == -1) return RobotDirections::LEFT;
        
    return RobotDirections::ANY;
}