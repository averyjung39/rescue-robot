#include <ros/Duration.h>
#include "objectives/ObjectiveManager.h"
#include "external/wiringPi/wiringPi.h"
#include "constants/gpio_pins.h"
//#include "constants/labels.h"

ObjectiveManager::ObjectiveManager() {
    // Activate FIND_FIRE and unativate all other objectives
    // FIND FIRE = 0, FIND_FOOD = 1, FIND_SURVIVORS = 2,
    // FIND_PERSON = 3, RETURN_HOME = 4
    _active_objectives = {true, false, false, false, false};
    setupGpio();
}

void ObjectiveManager::setupGpio() {
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }
    pinMode(INDICATOR_1, OUTPUT);
    pinMode(INDICATOR_2, OUTPUT);
    pinMode(INDICATOR_3, OUTPUT);
    pinMode(INDICATOR_4, OUTPUT);
    pinMode(FAN, OUTPUT);
}

void ObjectiveManager::activateObjectives(mapperception::Map map, int robot_i, int robot_j) {
    std::vector< std::vector<int> > label_map;
    for (int i = 0; i < map.size(); i++) {
        label_map.push_back(map[i]);
    }
    if (_active_objectives[objectives_list::FIND_FIRE]) {
        std::pair<int,int> fire_location = findObjectLocation(labels::FIRE)
        if (fire_location.first == -1) {
            // Fire hasn't been found or it has been put out
            fire_location = findObjectLocation(labels::NO_FIRE);
            if (fire_location.first != -1) {
                // Fire has been extinguished, turn off the fan
                turnFanOnOff(false);
            }
        } else {
            // Fire is located
            if (abs(robot_i - fire_location.first) == 1 && abs(robot_j - fire_location.second) == 1) {
                // robot and candle are right beside each other, turn on the fan
                turnFanOnOff(true);
            }
        }
    } else if () {

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

std::pair<int,int> ObjectiveManager::findObjectLocation(std::vector< std::vector<int> > label_map, int label) {
    for (int i = 0; i < label_map.size(); i++) {
        for (int j = 0; j < label_map.size(); j++) {
            if (label_map[i][j] == label) return std::make_pair(i,j);
        }
    }
    return std::make_pair(-1, -1);
}
