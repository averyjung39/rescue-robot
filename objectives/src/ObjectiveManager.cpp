#include <ros/duration.h>
#include "objectives/ObjectiveManager.h"
#include "external/wiringPi/wiringPi.h"
#include "constants/gpio_pins.h"
#include "constants/labels.h"
#include "constants/objectives_list.h"

ObjectiveManager::ObjectiveManager() {
    // Activate FIND_FIRE and unativate all other objectives
    // FIND FIRE = 0, FIND_FOOD = 1, FIND_SURVIVORS = 2,
    // FIND_PERSON = 3, RETURN_TO_SURVIVORS = 4, RETURN_HOME = 5
    _active_objectives.push_back(true);
    _active_objectives.push_back(false);
    _active_objectives.push_back(false);
    _active_objectives.push_back(false);
    _active_objectives.push_back(false);
    _active_objectives.push_back(false);
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

std::vector<bool> ObjectiveManager::activateObjectives(int robot_i, int robot_j, std::vector< std::vector<int> > label_map) {
    std::set<int> near_labels = getNearLabels(robot_i, robot_j, label_map);

    if (!near_labels.empty()) {
        if (_active_objectives[objectives_list::FIND_FIRE]) {
            if (near_labels.find(labels::FIRE) == near_labels.end()) {
                // Fire hasn't been found or it has been put out, or an error occured while calling service
                if (near_labels.find(labels::NO_FIRE) != near_labels.end()) {
                    // Fire has been extinguished, turn off the fan
                    digitalWrite(FAN, LOW);
                    // Deactivate FIND_FIRE objective, activate FIND_FOOD and FIND_PERSON objectives
                    _active_objectives[objectives_list::FIND_FIRE] = false;
                    _active_objectives[objectives_list::FIND_FOOD] = true;
                    _active_objectives[objectives_list::FIND_PERSON] = true;
                    _active_objectives[objectives_list::FIND_SURVIVORS] = true;
                    return _active_objectives;
                }
            } else {
                // Fire is located
                digitalWrite(FAN, HIGH);
                return _active_objectives;
            }
        }

        if (_active_objectives[objectives_list::FIND_FOOD]) {
            if (near_labels.find(labels::MAGNET) != near_labels.end()) {
                // Found food
                turnOnIndicator(FOOD_INDICATOR);
                // Deactivate FIND_FOOD objective, activate FIND_SURVIVORS objective
                _active_objectives[objectives_list::FIND_FOOD] = false;
                _active_objectives[objectives_list::RETURN_TO_SURVIVORS] = true;
                _active_objectives[objectives_list::FIND_SURVIVORS] = false;
                return _active_objectives;
            }
        }

        if (_active_objectives[objectives_list::FIND_PERSON]) {
            if (near_labels.find(labels::SMALL_HOUSE) != near_labels.end()) {
                // Found small house
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

        if (_active_objectives[objectives_list::FIND_SURVIVORS] ||
            _active_objectives[objectives_list::RETURN_TO_SURVIVORS])
        {
            if(near_labels.find(labels::BIG_HOUSE) != near_labels.end()) {
                // Found big house
                turnOnIndicator(SURVIVORS_INDICATOR);
                // Deactivate FIND_SURVIVORS objective
                _active_objectives[objectives_list::FIND_SURVIVORS] = false;
                _active_objectives[objectives_list::RETURN_TO_SURVIVORS] = false;
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

std::set<int> ObjectiveManager::getNearLabels(int robot_i, int robot_j, std::vector< std::vector<int> > map) {
    // {left, right, front, back}
    std::set<int> labels;
    if (robot_i != 0) {
        labels.insert(map[robot_i-1][robot_j]);
    }
    if (robot_i != 5) {
        labels.insert(map[robot_i+1][robot_j]);
    }
    if (robot_j != 5) {
        labels.insert(map[robot_i][robot_j+1]);
    }
    if (robot_j != 0) {
        labels.insert(map[robot_i][robot_j-1]);
    }
    return labels;
}

void ObjectiveManager::turnOnIndicator(int pin) {
    digitalWrite(pin, HIGH);
    ros::Duration(1).sleep();
    digitalWrite(pin, LOW);
}
