#include <ros/ros.h>
#include <math.h>
#include "constants/dimensions.h"
#include "constants/denoise_params.h"
#include "mapperception/Mapper.h"
#include "sensors/Distance.h"

Mapper::Mapper(int orientation) {
    _label_map.setLabel(5, 3, labels::FLAT_WOOD);
    ROS_INFO("ORIENTATION OF THE MAP: %d", orientation);
    if (orientation == 1) {
        _label_map.setLabel(5, 2, labels::PIT);
        _label_map.setLabel(4, 1, labels::SAND);
        _label_map.setLabel(4, 4, labels::GRAVEL);
        _label_map.setLabel(3, 5, labels::PIT);
        _label_map.setLabel(2, 0, labels::GRAVEL);
        _label_map.setLabel(2, 2, labels::SAND);
        _label_map.setLabel(1, 1, labels::PIT);
        _label_map.setLabel(1, 4, labels::SAND);
        _label_map.setLabel(0, 3, labels::GRAVEL);
    } else if (orientation == 2) {
        _label_map.setLabel(5, 2, labels::PIT);
        _label_map.setLabel(4, 1, labels::GRAVEL);
        _label_map.setLabel(4, 4, labels::SAND);
        _label_map.setLabel(3, 5, labels::GRAVEL);
        _label_map.setLabel(2, 0, labels::PIT);
        _label_map.setLabel(2, 3, labels::SAND);
        _label_map.setLabel(1, 1, labels::SAND);
        _label_map.setLabel(1, 4, labels::PIT);
        _label_map.setLabel(0, 3, labels::GRAVEL);
    } else if (orientation == 3) {
        _label_map.setLabel(5, 2, labels::GRAVEL);
        _label_map.setLabel(4, 1, labels::SAND);
        _label_map.setLabel(4, 4, labels::PIT);
        _label_map.setLabel(3, 3, labels::SAND);
        _label_map.setLabel(3, 5, labels::GRAVEL);
        _label_map.setLabel(2, 0, labels::PIT);
        _label_map.setLabel(1, 1, labels::GRAVEL);
        _label_map.setLabel(1, 4, labels::SAND);
        _label_map.setLabel(0, 3, labels::PIT);
    } else if (orientation == 4) {
        _label_map.setLabel(5, 2, labels::GRAVEL);
        _label_map.setLabel(4, 1, labels::PIT);
        _label_map.setLabel(4, 4, labels::SAND);
        _label_map.setLabel(3, 2, labels::SAND);
        _label_map.setLabel(3, 5, labels::PIT);
        _label_map.setLabel(2, 0, labels::GRAVEL);
        _label_map.setLabel(1, 1, labels::SAND);
        _label_map.setLabel(1, 4, labels::GRAVEL);
        _label_map.setLabel(0, 3, labels::PIT);
    }
}

void Mapper::setRobotPose(float robot_x, float robot_y, float robot_angle) {
    _robot_pos = std::make_pair(robot_x, robot_y);
    _robot_angle = robot_angle;
    std::pair<int,int> robot_points = coordinateToPoints(robot_x, robot_y, _label_map.getResolution());
    if (_label_map.queryMap(robot_points.first,robot_points.second) == labels::UNSEARCHED) {
        _label_map.setLabel(robot_points.first, robot_points.second, labels::FLAT_WOOD);
    }
}

void Mapper::modifyLabelMapWithDists(std::vector<float> dist_data, bool high_sensor)
{
    int label = high_sensor ? labels::TALL_OBJECT : labels::OBJECT;

    std::pair<int,int> robot_location = coordinateToPoints(_robot_pos.first, _robot_pos.second, _label_map.getResolution());

    for(int i = 0; i < dist_data.size(); i++) {
        // -1 is INVALID_SENSOR_DATA
        if (dist_data[i] == -1) continue;
        std::pair<float, float> coords = distToCoordinates(dist_data[i], _robot_pos.first, _robot_pos.second, _robot_angle, i, high_sensor);
        if (coords.first < denoise_params::WALL_OFFSET || coords.first > dimensions::MAP_WIDTH - denoise_params::WALL_OFFSET ||
            coords.second < denoise_params::WALL_OFFSET || coords.second > dimensions::MAP_HEIGHT - denoise_params::WALL_OFFSET)
        {
            // Too close to the wall, don't put it in the map
            return;
        }
        std::pair<int, int> points = coordinateToPoints(coords.first, coords.second, _label_map.getResolution());
        // Put the object in the label map if it's not on the robot position
        if (points.first != robot_location.first && points.second != robot_location.second) {
            int map_label = _label_map.queryMap(points.first,points.second);
            if (high_sensor) {
                // label it as TALL_OBJECT only if the cell is labeled as OBJECT or UNSEARCHED or FLAT_WOOD
                if (map_label == labels::OBJECT || map_label == labels::UNSEARCHED || map_label == labels::FLAT_WOOD) {
                    _label_map.setLabel(points.first, points.second, label);
                }
            } else {
                // label it as OBJECT only if the cell is labeled as UNSEARCHED
                if (map_label == labels::UNSEARCHED || map_label == labels::FLAT_WOOD) {
                    _label_map.setLabel(points.first, points.second, label);
                }
            }
            // Mark the tiles between the robot and the object as FLAT_WOOD unless they are different terrain
            if (_robot_angle > 80 && _robot_angle < 100) {
                for(int i = robot_location.first-1; i > points.first; i--) {
                    if (_label_map.queryMap(i, points.second) != labels::UNSEARCHED) continue;
                    _label_map.setLabel(i, points.second, labels::FLAT_WOOD);
                }
            } else if (_robot_angle > 170 && _robot_angle < 190) {
                for(int j = robot_location.second-1; j > points.second; j--) {
                    if (_label_map.queryMap(points.first,j) != labels::UNSEARCHED) continue;
                    _label_map.setLabel(points.first, j, labels::FLAT_WOOD);
                }
            } else if (_robot_angle > 260 && _robot_angle < 280) {
                for(int i = robot_location.first+1; i < points.first; i++) {
                    if (_label_map.queryMap(i, points.second) != labels::UNSEARCHED) continue;
                    _label_map.setLabel(i, points.second, labels::FLAT_WOOD);
                }
            } else if (_robot_angle > 350 || _robot_angle < 10) {
                for(int j = robot_location.second+1; j < points.second; j++) {
                    if (_label_map.queryMap(points.first, j) != labels::UNSEARCHED) continue;
                    _label_map.setLabel(points.first, j, labels::FLAT_WOOD);
                }
            }
        }
    }
}

bool Mapper::detectFire(std::vector<int> photodiode_data) {
    // Check if candle has been detected, try classifying the object if candle hasn't been found
    for(int i = 0; i < photodiode_data.size(); i++) {
        if (photodiode_data[i]) {
            return true;
        }
    }
    // none of the photodiodes see fire
    return false;
}

void Mapper::modifyLabelMapWithPhotodiode(std::vector<int> photodiode_data) {
    if (_found_labels.find(labels::FIRE) != _found_labels.end()) {
        std::pair<int,int> fire_location = indicesInFront();
        int map_label = _label_map.queryMap(fire_location.first, fire_location.second);
        if (map_label == labels::FIRE) {
            for(int i = 0; i < photodiode_data.size(); i++) {
                // If any of the photodiode sensors is high, then fire hasn't been extinguished
                if (photodiode_data[i]) return;
            }
            _found_labels.insert(labels::NO_FIRE);
            _label_map.setLabel(fire_location.first, fire_location.second, labels::NO_FIRE);
        }
    }
}

bool Mapper::detectHouses(float dist_l, float dist_r) {
    bool big_house_detected = false;
    if (dist_l < 50.0 && dist_l != sensors::Distance::INVALID_SENSOR_DATA) {
        // Check if the x,y point of the ToF reading is too close to the wall
        std::pair<float,float> coords =
            distToCoordinates(dist_l, _robot_pos.first, _robot_pos.second, _robot_angle, TOP_LEFT, true);

        if (coords.first < denoise_params::WALL_OFFSET || coords.first > dimensions::MAP_WIDTH - denoise_params::WALL_OFFSET ||
            coords.second < denoise_params::WALL_OFFSET || coords.second > dimensions::MAP_HEIGHT - denoise_params::WALL_OFFSET)
        {
            // Too close to the wall, return false to indicate it's not the big house
            big_house_detected = false;
        } else {
            big_house_detected = true;
        }
    }
    if (!big_house_detected) {
        if (dist_r < 50.0 && dist_r != sensors::Distance::INVALID_SENSOR_DATA) {
            // Check if the x,y point of the ToF reading is too close to the wall
            std::pair<float,float> coords =
                distToCoordinates(dist_r, _robot_pos.first, _robot_pos.second, _robot_angle, TOP_RIGHT, true);

            if (coords.first < denoise_params::WALL_OFFSET || coords.first > dimensions::MAP_WIDTH - denoise_params::WALL_OFFSET ||
                coords.second < denoise_params::WALL_OFFSET || coords.second > dimensions::MAP_HEIGHT - denoise_params::WALL_OFFSET)
            {
                // Too close to the wall, return false to indicate it's not the big house
                big_house_detected = false;
            } else {
                big_house_detected = true;
            }
        }
    }
    return big_house_detected;
}

void Mapper::updateLabelMapWithScanningResults(bool &big_house_detected, bool &fire_detected) {
    std::pair<int,int> object_location = indicesInFront();
    int map_label = _label_map.queryMap(object_location.first, object_location.second);

    if (map_label == labels::TALL_OBJECT || map_label == labels::OBJECT) {
        if (fire_detected) {
            big_house_detected = false;
            if (_found_labels.find(labels::FIRE) == _found_labels.end()) {
                _found_labels.insert(labels::FIRE);
                _label_map.setLabel(object_location.first, object_location.second, labels::FIRE);
                return;
            }
        }

        if (big_house_detected) {
            if (_found_labels.find(labels::BIG_HOUSE) == _found_labels.end()) {
                _found_labels.insert(labels::BIG_HOUSE);
                _label_map.setLabel(object_location.first, object_location.second, labels::BIG_HOUSE);
            }
        } else {
            if (_found_labels.find(labels::SMALL_HOUSE) == _found_labels.end()) {
                _found_labels.insert(labels::SMALL_HOUSE);
                _label_map.setLabel(object_location.first, object_location.second, labels::SMALL_HOUSE);
            }
        }
    } else {
        big_house_detected = false;
        fire_detected = false;
    }
}

void Mapper::detectMagnet(bool hall_effect_data) {
    std::pair<int,int> magnet_location = coordinateToPoints(_robot_pos.first, _robot_pos.second, _label_map.getResolution());
    int map_label = _label_map.queryMap(magnet_location.first, magnet_location.second);
    // label the indices as MAGNET if:
    // 1. magnet hasn't been found and
    // 2. hall effect detected magnet and
    // 3. the indices are labeled as SAND
    if (_found_labels.find(labels::MAGNET) == _found_labels.end() && hall_effect_data &&
        (map_label == labels::SAND || map_label == labels::NO_MAGNET)) {
        _found_labels.insert(labels::MAGNET);
        _label_map.setLabel(magnet_location.first, magnet_location.second, labels::MAGNET);
    } else if (!hall_effect_data && map_label == labels::SAND) {
        _label_map.setLabel(magnet_location.first, magnet_location.second, labels::NO_MAGNET);
    }
}

std::pair<float, float> Mapper::distToCoordinates(float d, float rx, float ry, float rangle, int sensor, bool high_sensor) {
    // xr,yr points in local robot axes
    float xr = 0.0;
    float yr = 0.0;
    if (!high_sensor) {
        switch(sensor) {
            case BOTTOM_LEFT:
                xr = d+dimensions::BOTTOM_TOF_X_OFFSET;
                yr = dimensions::BOTTOM_TOF_Y_OFFSET;
                break;
            case BOTTOM_RIGHT:
                xr = d+dimensions::BOTTOM_TOF_X_OFFSET;
                yr = -(dimensions::BOTTOM_TOF_Y_OFFSET);
                break;
            default:
                ROS_ERROR("Unknown sensor type.");
                break;
        }
    } else {
        switch(sensor) {
            case TOP_FRONT:
                xr = d+dimensions::TOP_TOF_FRONT_X_OFFSET;
                yr = dimensions::TOP_TOF_FRONT_Y_OFFSET;
                break;
            case TOP_BACK:
                xr = -(d+dimensions::TOP_TOF_OFFSET);
                yr = dimensions::TOP_TOF_BACK_Y_OFFSET;
                break;
            case TOP_LEFT:
                yr = d+dimensions::TOP_TOF_OFFSET;
                xr = dimensions::TOP_TOF_LEFT_X_OFFSET;
                break;
            case TOP_RIGHT:
                yr = -(d+dimensions::TOP_TOF_OFFSET);
                xr = dimensions::TOP_TOF_RIGHT_X_OFFSET;
                break;
            default:
                ROS_ERROR("Unknown sensor type.");
                break;
        }
    }

    float rad_angle = rangle*M_PI/180;
    // Convert xr,yr (local robot axes) to x,y (global axes)
    int x = rx + xr*cos(rangle) + yr*sin(rad_angle);
    int y = ry - xr*sin(rangle) + yr*cos(rad_angle);

    return std::make_pair(x,y);
}

std::pair<int,int> Mapper::coordinateToPoints(float x, float y, int resolution) {
    float cm_per_px = dimensions::TILE_WIDTH_CM/resolution;
    int row = _label_map.getSize() - floor(y/cm_per_px) - 1;
    int col = floor(x/cm_per_px);
    return std::make_pair(row, col);
}

std::pair<int,int> Mapper::indicesInFront() {
    std::pair<int,int> robot_points = coordinateToPoints(_robot_pos.first, _robot_pos.second, _label_map.getResolution());
    if (_robot_angle > 80 && _robot_angle < 100) {
        return std::make_pair(robot_points.first + 1, robot_points.second);
    } else if (_robot_angle > 170 && _robot_angle < 190) {
        return std::make_pair(robot_points.first, robot_points.second - 1);
    } else if (_robot_angle > 260 && _robot_angle < 280) {
        return std::make_pair(robot_points.first - 1, robot_points.second);
    } else if (_robot_angle > 350 || _robot_angle < 10) {
        return std::make_pair(robot_points.first, robot_points.second + 1);
    } else {
        return std::make_pair(-1, -1);
    }
}
