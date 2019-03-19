#include <ros/ros.h>
#include <math.h>

#include "mapperception/Mapper.h"
#include "sensors/Distance.h"

Mapper::Mapper() {
    _object_rows.insert(std::pair<int, std::vector<int> >(labels::MAGNET, std::vector<int>()));
    _object_rows.insert(std::pair<int, std::vector<int> >(labels::FIRE, std::vector<int>()));
    _object_rows.insert(std::pair<int, std::vector<int> >(labels::NO_FIRE, std::vector<int>()));
    _object_rows.insert(std::pair<int, std::vector<int> >(labels::SMALL_HOUSE, std::vector<int>()));
    _object_rows.insert(std::pair<int, std::vector<int> >(labels::BIG_HOUSE, std::vector<int>()));

    _object_cols.insert(std::pair<int, std::vector<int> >(labels::MAGNET, std::vector<int>()));
    _object_cols.insert(std::pair<int, std::vector<int> >(labels::FIRE, std::vector<int>()));
    _object_cols.insert(std::pair<int, std::vector<int> >(labels::NO_FIRE, std::vector<int>()));
    _object_cols.insert(std::pair<int, std::vector<int> >(labels::SMALL_HOUSE, std::vector<int>()));
    _object_cols.insert(std::pair<int, std::vector<int> >(labels::BIG_HOUSE, std::vector<int>()));

    // TODO: hardcoded value of 25 cm, measure what we actually see from ultrasonic on the flat ground
    _prev_ult_data = 25;
}

bool Mapper::getObjectLocs(mapperception::ObjectLocation::Request &req, mapperception::ObjectLocation::Response &res) {
    res.row_vector = _object_rows.at(req.label);
    res.col_vector = _object_cols.at(req.label);
    return true;
}

void Mapper::modifyLabelMapWithDists(std::vector<float> dist_data,
                                     float robot_x, float robot_y,
                                     float robot_angle) {
    for(int i = 0; i < dist_data.size(); i++) {
        // -1 is INVALID_SENSOR_DATA
        if (dist_data[i] == -1) continue;
        std::pair<float, float> coords = distToCoordinates(dist_data[i], robot_x, robot_y, robot_angle, i);
        std::pair<int, int> points = coordinateToPoints(coords.first, coords.second, _label_map.getResolution());
        if (points.first > 5 || points.second > 5 || points.first < 0 || points.second < 0) {
            ROS_WARN("Out of bound {%d, %d}", points.first, points.second);
            continue;
        }
        _label_map.setLabel(points.first, points.second, labels::OBJECT);

    }
}

void Mapper::modifyLabelMapWithLabels(int row, int col, int label) {
    if (row > 5 || col > 5 || row < 0 || col < 0) {
        ROS_WARN("Out of bound {%d, %d}", row, col);
    } else {
        _label_map.setLabel(row, col, label);
        _object_rows[label].push_back(row);
        _object_cols[label].push_back(col);
    }
}

int Mapper::classifyObject(std::vector<float> high_dist_data, std::vector<bool> photodiode_data)
{
    // Check if candle has been detected, try classifying the object if candle hasn't been found
    if (_object_rows.at(labels::FIRE).empty()) {
        for(int i = 0; i < photodiode_data.size(); i++) {
            if (photodiode_data[i]) return labels::FIRE;
        }
    } else if (_object_rows.at(labels::NO_FIRE).empty()) {
        // Assume we are putting the fire out right after detecting it
        for(int i = 0; i < photodiode_data.size(); i++) {
            // If any of the photodiode sensors is high, then fire hasn't been extinguished
            if (photodiode_data[i]) return labels::FIRE;
        }
        // Fire has been extinguished, return NO_FIRE label
        return labels::NO_FIRE;
    }

    if (_object_rows.at(labels::BIG_HOUSE).empty()) {
        // Check if the distance data is too far (greater than 50 cm),
        // if it is then it's not being used for classification
        // Note: back tof sensor data is not being used for classification
        if (((high_dist_data[0] < 50.0) && high_dist_data[0] != sensors::Distance::INVALID_SENSOR_DATA)
            || ((high_dist_data[2] < 50.0) && high_dist_data[2] != sensors::Distance::INVALID_SENSOR_DATA))
        {
            return labels::BIG_HOUSE
        }
    }
}

void Mapper::detectTerrain(float ultrasonic_data, std::vector<bool> colour_data, bool hall_effect_data,
                           float robot_x, float robot_y, float robot_angle) {
    _robot_pos = coordinateToPoints(robot_x, robot_y, _label_map.getResolution());
    // TODO: hardcoded difference of 5 cm, measure what utrasonic actually sees from the pit to determine this difference
    int label = 0;
    if(abs(_prev_ult_data - ultrasonic_data) > 5) {
        // Detected the pit
        label = labels::PIT;
    } else if (colour_data[0]) {
        // SAND, check if there is magnet
        if (hall_effect_data) {
            label = labels::MAGNET;
        } else {
            label = labels::SAND;
        }
    } else if (colour_data[1]) {
        // ROCK
        label = labels::ROCK;
    }
    if (label) {
        // Detected a terrain
        if (robot_angle > 80 && robot_angle < 100) {
            modifyLabelMapWithLabels(_robot_pos.first + 1, _robot_pos.second, label);
        } else if (robot_angle > 170 && robot_angle < 190) {
            modifyLabelMapWithLabels(_robot_pos.first, _robot_pos.second - 1, label);
        } else if (robot_angle > 260 && robot_angle < 280) {
            modifyLabelMapWithLabels(_robot_pos.first - 1, _robot_pos.second, label);
        } else if (robot_angle > 350 || robot_angle < 10) {
            modifyLabelMapWithLabels(_robot_pos.first, _robot_pos.second + 1, label);
        }
    }
}

std::pair<float, float> Mapper::distToCoordinates(float d, float rx, float ry, float rangle, int sensor) {
    // xr,yr points in local robot axes
    float xr = 0.0;
    float yr = 0.0;

    switch(sensor) {
        case LEFT:
            xr = d+BOTTOM_TOF_Y_OFFSET;
            yr = -BOTTOM_TOF_X_OFFSET;
            break;
        case MIDDLE:
            xr = d+BOTTOM_TOF_Y_OFFSET;
            break;
        case RIGHT:
            xr = d+BOTTOM_TOF_Y_OFFSET;
            yr = BOTTOM_TOF_X_OFFSET;
            break;
        default:
            ROS_ERROR("Unknown sensor type.");
            break;
    }
    float rad_angle = rangle*M_PI/180;
    // Convert xr,yr (local robot axes) to x,y (global axes)
    int x = rx + xr*cos(rangle) + yr*sin(rad_angle);
    int y = ry - xr*sin(rangle) + yr*cos(rad_angle);

    return std::make_pair(x,y);
}

std::pair<int, int> Mapper::coordinateToPoints(float x, float y, int resolution) {
    float cm_per_px = 30.48/resolution;
    int row = _label_map.getSize() - floor(y/cm_per_px) - 1;
    int col = floor(x/cm_per_px);
    return std::make_pair(row, col);
}
