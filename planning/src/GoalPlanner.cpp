#include "constants/objectives_list.h"
#include "constants/labels.h"
#include "planning/GoalPlanner.h"

GoalPlanner::GoalPlanner() {
    _run_initialization = true;
    _home_indices = std::make_pair(5,3);
}

GoalPose GoalPlanner::getGoal(
    const localization::Pose &pose,
    const mapperception::Map &map,
    const objectives::ActiveObjectives &objectives) {
    
    // TODO: Incorporate timeouts for some of these objectives

    GoalPose goal_pose;
    int robot_i = map.robot_i;
    int robot_j = map.robot_j;
    // At the very beginning, sweep left and right
    if (_run_initialization) {
        if (pose.theta < 180) {
            // Keep turning left. Set GoalPose to current indices in map, LEFT
            goal_pose.i = robot_i;
            goal_pose.j = robot_j;
            goal_pose.direction = ROBOT_DIRECTION_LEFT;
        } else if (pose.theta > 0 && pose.theta < 270) {
            // Start turning right. Set GoalPose to current indices in map, RIGHT
            goal_pose.i = robot_i;
            goal_pose.j = robot_j;
            goal_pose.direction = ROBOT_DIRECTION_RIGHT;
        } else {
            _run_initialization = false;
        }
    } else {
        std::vector<int> active_objectives = objectives.active_objectives;
        
        // Start checking objectives
        // Want to find fire first, so check that first
        std::pair<int, int> map_search_result;
        std::pair<int, int> goal_indices;
        if (active_objectives[objectives_list::FIND_FIRE]) {
            // If there is at least one unknown obstacle in the map, set GoalPose to indices adjacent to closest obstacle, direction facing obstacle
            // Else set goal to some other area, like the fathest unsearched area or farthest edge of map
            
            if (searchMap(map, labels::TALL_OBJECT, robot_i, robot_j, map_search_result) ||
                searchMap(map, labels::OBJECT, robot_i, robot_j, map_search_result)) {
                goal_indices = lowestCostIndicesNearby(map, map_search_result.first, map_search_result.second, robot_i, robot_j);
                goal_pose.direction = getDirectionFacingCell(goal_indices, map_search_result);
            } else if (farthestUnsearchedCell(map, robot_i, robot_j, map_search_result)) {
                goal_indices = map_search_result;
                goal_pose.direction = ROBOT_DIRECTION_ANY;
            } else {
                goal_indices = farthestEdgeFromRobot(robot_i, robot_j);
                goal_pose.direction = ROBOT_DIRECTION_ANY;
            }
        }
        // If we want to return home, this will be the only active objective, so we should just return home 
        else if (active_objectives[objectives_list::RETURN_HOME]) {
            // Set goal to home indices, orientation ANY
            goal_indices = _home_indices;
            goal_pose.direction = ROBOT_DIRECTION_ANY;
        } 
        // For all other active objectives, check if any of their locations have already been found in the map
        else if (active_objectives[objectives_list::FIND_FOOD] && searchMap(map, labels::MAGNET, robot_i, robot_j, map_search_result)) {
            // Set goal to indices of closest magnet, orientation ANY
            goal_indices = map_search_result;
            goal_pose.direction = ROBOT_DIRECTION_ANY;
        } else if ((active_objectives[objectives_list::FIND_SURVIVORS] || active_objectives[objectives_list::RETURN_TO_SURVIVORS]) 
            && searchMap(map, labels::BIG_HOUSE, robot_i, robot_j, map_search_result)) {
            // Set goal to indices adjacent to survivors cell, direction facing survivors
            goal_indices = lowestCostIndicesNearby(map, map_search_result.first, map_search_result.second, robot_i, robot_j);
            goal_pose.direction = getDirectionFacingCell(goal_indices, map_search_result);
        } else if (active_objectives[objectives_list::FIND_PERSON] && searchMap(map, labels::SMALL_HOUSE, robot_i, robot_j, map_search_result)) {
            // Set goal to indices adjacent to person cell, direction facing person
            goal_indices = lowestCostIndicesNearby(map, map_search_result.first, map_search_result.second, robot_i, robot_j);
            goal_pose.direction = getDirectionFacingCell(goal_indices, map_search_result);
        }
        // If we haven't found the locations of these objectives, check unknown obstacles or unsearched areas
        else if (active_objectives[objectives_list::FIND_PERSON] && searchMap(map, labels::OBJECT, robot_i, robot_j, map_search_result)) {
            goal_indices = lowestCostIndicesNearby(map, map_search_result.first, map_search_result.second, robot_i, robot_j);
            goal_pose.direction = getDirectionFacingCell(goal_indices, map_search_result);
        } else if ((active_objectives[objectives_list::RETURN_TO_SURVIVORS] || active_objectives[objectives_list::FIND_SURVIVORS])
            && (searchMap(map, labels::TALL_OBJECT, robot_i, robot_j, map_search_result) ||
            searchMap(map, labels::OBJECT, robot_i, robot_j, map_search_result))) {
            // Set goal to indices adjacent to closest unknown obstacle, direction facing obstacle
            goal_indices = lowestCostIndicesNearby(map, map_search_result.first, map_search_result.second, robot_i, robot_j);
            goal_pose.direction = getDirectionFacingCell(goal_indices, map_search_result);
        } else if (active_objectives[objectives_list::FIND_FOOD] && searchMap(map, labels::SAND, robot_i, robot_j, map_search_result)){
            goal_indices = map_search_result;
            goal_pose.direction = ROBOT_DIRECTION_ANY;
        } else if (farthestUnsearchedCell(map, robot_i, robot_j, map_search_result)) {
            // Set goal to indices of farthest unsearched space, direction ANY
            goal_indices = map_search_result;
            goal_pose.direction = ROBOT_DIRECTION_ANY;
        } else {
            // Not sure what to do if we get to here. Probably print an error message and either spin on the spot or
            // try re-searching areas

            // Alternatively, we send a signal to objectives saying we weren't able to complete whatever objectives are currently listed
            // as active. Objectives then sends us a new list of objectives to do. (Implementing this would solve the issue of not having timeouts)

            // For now just go to the farthest edge of the map and hope that we see something new
            goal_indices = farthestEdgeFromRobot(robot_i, robot_j);
            goal_pose.direction = ROBOT_DIRECTION_ANY;
        }
        goal_pose.i = goal_indices.first;
        goal_pose.j = goal_indices.second;
    }

    return goal_pose;
}

RobotDirections GoalPlanner::getDirectionFacingCell(const std::pair<int, int> &curr_indices,
    const std::pair<int, int> &goal_indices) const {
    int i_diff = goal_indices.first - curr_indices.first;
    int j_diff = goal_indices.second - curr_indices.second;

    if (i_diff == 1 && j_diff == 0) return ROBOT_DIRECTION_DOWN;
    else if (i_diff == -1 && j_diff == 0) return ROBOT_DIRECTION_UP;
    else if (i_diff == 0 && j_diff == 1) return ROBOT_DIRECTION_RIGHT;
    else if (i_diff == 0 && j_diff == -1) return ROBOT_DIRECTION_LEFT;
        
    return ROBOT_DIRECTION_ANY;
}

bool GoalPlanner::searchMap(const mapperception::Map &map, const int &label, const int &robot_i, const int &robot_j, std::pair<int, int> &closest_indices) const {
    std::vector<std::pair<int, int> > indices;
    for (int i = 0; i < map.map.size(); ++i) {
        std::vector<int> row = map.map[i].row;
        for (int j = 0; j < row.size(); ++j) {
            if (row[j] == label) {
                indices.push_back(std::make_pair(i,j));
            }
        }
    }

    float min_distance = 100000;
    for (int i = 0; i < indices.size(); ++i) {
        float distance = sqrt((indices[i].first - robot_i)*(indices[i].first - robot_i) + (indices[i].second - robot_j)*(indices[i].second - robot_j));
        if (distance < min_distance) {
            min_distance = distance;
            closest_indices = indices[i];
        }
    }
    return indices.size() > 0;
}

std::pair<int, int> GoalPlanner::lowestCostIndicesNearby(const mapperception::Map &map,
    const int &i,
    const int &j,
    const int &robot_i,
    const int &robot_j) const {
    std::pair<int, int> indices;
    std::vector<std::pair<int, int> > indices_to_try;
    if (i < map.map.size()-1) {
        indices_to_try.push_back(std::make_pair(i+1,j));
    }
    if (i > 0) {
        indices_to_try.push_back(std::make_pair(i-1,j));
    }
    if (j < map.map.size()-1) {
        indices_to_try.push_back(std::make_pair(i,j+1));
    }
    if (j > 0) {
        indices_to_try.push_back(std::make_pair(i,j-1));
    }

    float min_cost = 1000000;
    for (int k = 0; k < indices_to_try.size(); ++k) {
        int i_to_try = indices_to_try[k].first;
        int j_to_try = indices_to_try[k].second;
        float cost = map.map[i_to_try].row[j_to_try] + sqrt((i_to_try - robot_i)*(i_to_try - robot_i) + (j_to_try - robot_j)*(j_to_try - robot_j));
        if (cost < min_cost) {
            indices = indices_to_try[k];
            min_cost = cost;
        }
    }

    return indices;
}

std::pair<int, int> GoalPlanner::farthestEdgeFromRobot(const int &robot_i, const int &robot_j) const {
    std::vector<std::pair<int, int> > indices_to_try;
    indices_to_try.push_back(std::make_pair(0,2)); // All different possible start locations
    indices_to_try.push_back(std::make_pair(2,5));
    indices_to_try.push_back(std::make_pair(3,0));
    indices_to_try.push_back(std::make_pair(5,3));

    float max_distance = 0;
    std::pair<int, int> indices;
    for (int i = 0; i < indices_to_try.size(); ++i) {
        float distance = sqrt((indices_to_try[i].first - robot_i)*(indices_to_try[i].first - robot_i) + (indices_to_try[i].second - robot_j)*(indices_to_try[i].second - robot_j));
        if (distance > max_distance) {
            max_distance = distance;
            indices = indices_to_try[i];
        }
    }
    return indices;
}

bool GoalPlanner::farthestUnsearchedCell(
    const mapperception::Map &map,
    const int &robot_i,
    const int &robot_j,
    std::pair<int, int> &indices) const {
    float max_distance = 0;
    bool found = false;
    for (int i = 0; i < map.map.size(); ++i) {
        std::vector<int> row = map.map[i].row;
        for (int j = 0; j < row.size(); ++j) {
            if (row[j] == labels::UNSEARCHED) {
                found = true;
                float distance = sqrt((i - robot_i)*(i - robot_i) + (j - robot_j)*(j - robot_j));
                if (distance > max_distance) {
                    max_distance = distance;
                    indices = std::make_pair(i,j);
                }
            }
        }
    }
    return found;
}