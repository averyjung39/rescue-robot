#include <ros/duration.h>
#include "objectives/ObjectiveManager.h"
#include "external/wiringPi/wiringPi.h"
#include "constants/gpio_pins.h"
#include "constants/labels.h"
#include "constants/objectives_list.h"

ObjectiveManager::ObjectiveManager(ros::NodeHandle& nh, const char* service_name) {
    // Activate FIND_FIRE and unativate all other objectives
    // FIND FIRE = 0, FIND_FOOD = 1, FIND_SURVIVORS = 2,
    // FIND_PERSON = 3, RETURN_HOME = 4
    _active_objectives.push_back(true);
    _active_objectives.push_back(false);
    _active_objectives.push_back(false);
    _active_objectives.push_back(false);
    _active_objectives.push_back(false);
    _obj_loc_client = nh.serviceClient<mapperception::ObjectLocation>(service_name);
    setupGpio();
}

void ObjectiveManager::setupGpio() {
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }
    pinMode(FOOD_INDICATOR, OUTPUT);
    pinMode(PERSON_INDICATOR, OUTPUT);
    pinMode(SURVIVORS_INDICATOR, OUTPUT);
    pinMode(HOME_INDICATOR, OUTPUT);
    pinMode(FAN, OUTPUT);
}

std::pair< std::vector<int>, std::vector<int> > ObjectiveManager::findObjectLocation(int label) {
    _srv.request.label = label;
    if (_obj_loc_client.call(_srv)) {
        return std::make_pair(_srv.response.row_vector, _srv.response.col_vector);
    } else {
        ROS_ERROR("Failed to call service object_location");
        return std::make_pair(std::vector<int>(), std::vector<int>());
    }
}

std::vector<bool> ObjectiveManager::activateObjectives(int robot_i, int robot_j) {
    std::pair< std::vector<int>, std::vector<int> > obj_loc;
    std::vector<int> obj_row;
    std::vector<int> obj_col;

    if (_active_objectives[objectives_list::FIND_FIRE]) {
        obj_loc = findObjectLocation(labels::FIRE);
        obj_row = obj_loc.first;
        obj_col = obj_loc.second;

        if (obj_row.empty()) {
            // Fire hasn't been found or it has been put out, or an error occured while calling service
            obj_loc = findObjectLocation(labels::NO_FIRE);
            obj_row = obj_loc.first;
            obj_col = obj_loc.second;
            if (!obj_row.empty()) {
                // Fire has been extinguished, turn off the fan
                turnFanOnOff(false);
                // Deactivate FIND_FIRE objective, activate FIND_FOOD and FIND_PERSON objectives
                _active_objectives[objectives_list::FIND_FIRE] = false;
                _active_objectives[objectives_list::FIND_FOOD] = true;
                _active_objectives[objectives_list::FIND_PERSON] = true;
                return _active_objectives;
            }
        } else {
            // Fire is located
            // Note: assume the object is in 1 tile for now
            if (abs(robot_i - obj_row[0]) == 1 && abs(robot_j - obj_col[0]) == 1) {
                // robot and candle are right beside each other, turn on the fan
                turnFanOnOff(true);
            }
            return _active_objectives;
        }
    }

    if (_active_objectives[objectives_list::FIND_FOOD]) {
        obj_loc = findObjectLocation(labels::MAGNET);
        obj_row = obj_loc.first;
        obj_col = obj_loc.second;

        if (!obj_row.empty()) {
            // Found food, check if we are near the food
            if (abs(robot_i - obj_row[0]) <= 1 && abs(robot_j - obj_col[0]) <= 1) {
                turnOnIndicator(FOOD_INDICATOR);
                // Deactivate FIND_FOOD objective, activate FIND_SURVIVORS objective
                _active_objectives[objectives_list::FIND_FOOD] = false;
                _active_objectives[objectives_list::FIND_SURVIVORS] = true;
                return _active_objectives;
            }
        }
    }

    if (_active_objectives[objectives_list::FIND_PERSON]) {
        obj_loc = findObjectLocation(labels::SMALL_HOUSE);
        obj_row = obj_loc.first;
        obj_col = obj_loc.second;

        if (!obj_row.empty()) {
            // Found small house, check if we are near the small house
            if (abs(robot_i - obj_row[0]) == 1 && abs(robot_j - obj_col[0]) == 1) {
                turnOnIndicator(PERSON_INDICATOR);
                // Deactivate FIND_PERSON objective
                _active_objectives[objectives_list::FIND_PERSON] = false;
                // Check if all the missions were completed, if so, return to home
                for (int i = 0; i < _active_objectives.size(); i++) {
                    // If there are any uncompleted missions, just return the active objectives list
                    if (_active_objectives[i]) return _active_objectives;
                }
                _active_objectives[objectives_list::RETURN_HOME] = true;
                return _active_objectives;
            }
        }
    }

    if (_active_objectives[objectives_list::FIND_SURVIVORS]) {
        obj_loc = findObjectLocation(labels::BIG_HOUSE);
        obj_row = obj_loc.first;
        obj_col = obj_loc.second;

        if (!obj_row.empty()) {
            // Found big house, check if we are near the big house
            if (abs(robot_i - obj_row[0]) == 1 && abs(robot_j - obj_col[0]) == 1) {
                turnOnIndicator(SURVIVORS_INDICATOR);
                // Deactivate FIND_SURVIVORS objective
                _active_objectives[objectives_list::FIND_SURVIVORS] = false;
                // Check if all the missions were completed, if so, return to home
                for (int i = 0; i < _active_objectives.size(); i++) {
                    // If there are any uncompleted missions, just return the active objectives list
                    if (_active_objectives[i]) return _active_objectives;
                }
                _active_objectives[objectives_list::RETURN_HOME] = true;
                return _active_objectives;
            }
        }
    }
}

void ObjectiveManager::turnFanOnOff(bool on) {
    if (on) {
        digitalWrite(FAN, HIGH);
    } else {
        digitalWrite(FAN, LOW);
    }
}

void ObjectiveManager::turnOnIndicator(int pin) {
    digitalWrite(pin, HIGH);
    ros::Duration(1).sleep();
    digitalWrite(pin, LOW);
}
