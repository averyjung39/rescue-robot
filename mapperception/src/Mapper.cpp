#include "mapperception/Mapper.h"
#include <ros/ros.h>
#include <math.h>

void Mapper::modifyLabelMapWithDists(std::vector<float> dist_data,
                                     float robot_x, float robot_y,
                                     float robot_angle) {
    for(int i = 0; i < dist_data.size(); i++) {
        // TODO: replace -1 with INVALID_SENSOR_DATA
        if (dist_data[i] == -1) continue;
        std::pair<float, float> coords = distToCoordinates(dist_data[i], robot_x, robot_y, robot_angle, i);
        std::pair<int, int> points = coordinateToPoints(coords.first, coords.second, _label_map.getResolution());
        _label_map.setLabel(points.first, points.second, labels::OBJECT);
    }
}

void Mapper::modifyLabelMapWithLabels(int robot_i, int robot_j, int label) {
    _label_map.setLabel(robot_i, robot_j, label);
}

std::pair<int, int> Mapper::robotPosToPoints(float robot_x, float robot_y) {
    return coordinateToPoints(robot_x, robot_y, _label_map.getResolution);
}

std::pair<float, float> Mapper::distToCoordinates(float d, float rx, float ry, float rangle, int sensor) {
    // xr,yr points in local robot axes
    float xr = 0.0;
    float yr = 0.0;

    switch(sensor) {
        case LEFT:
            xr = -TOF24_X_OFFSET;
            yr = d+TOF234_Y_OFFSET;
            break;
        case MIDDLE:
            yr = d+TOF234_Y_OFFSET;
            break;
        case RIGHT:
            xr = TOF24_X_OFFSET;
            yr = d+TOF234_Y_OFFSET;
            break;
        default:
            ROS_ERROR("Unknown sensor type.");
            break;
    }

    // Convert xr,yr (local robot axes) to x,y (global axes)
    int x = rx + xr*cos(rangle) + yr*sin(rangle);
    int y = ry - xr*sin(rangle) + yr*cos(rangle);

    return std::make_pair(x,y);
}

std::pair<int, int> Mapper::coordinateToPoints(float x, float y, int resolution) {
    float cm_per_px = 30.48/resolution;
    int row = ceil(y/cm_per_px);
    int col = ceil(x/cm_per_px);
    return std::make_pair(row, col);
}
