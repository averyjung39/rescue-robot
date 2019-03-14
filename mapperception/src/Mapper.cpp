#include "mapperception/Mapper.h"
#include <ros/ros.h>
#include <math.h>

std::vector< std::vector<int> > Mapper::modifyCostMap(std::vector<float> dists, float robot_x, float robot_y, float robot_angle) {
    for(int i = robot_x-RADIUS; i <= robot_x+RADIUS; i++) {
        for(int j = robot_y-RADIUS; j <= robot_y+RADIUS; j++) {
            if(i < 0 || j < 0) continue;
            _cost_map.setValue(i, j, 50);
        }
    }

    for(int i = 0; i < dists.size(); i++) {
        int sensor;
        if (dists.size() == 5) {
            sensor = i;
        } else {
            sensor = 5+i;
        }
        if (dists[i] == -1) continue;
        std::pair<int,int> coords = distToCoordinates(dists[i], robot_x, robot_y, robot_angle, sensor);
        std::pair<int,int> points = coordinateToPoints(coords.first, coords.second, _labeled_map.getResolution());
        _cost_map.setValue(points.first, points.second, 100);
    }
    return _cost_map.getMap();
}

std::vector< std::vector<int> > Mapper::modifyLabeledMap(float x, float y, Labels label) {
    std::pair<int,int> points = coordinateToPoints(x, y, _labeled_map.getResolution());
    _labeled_map.setValue(points.first, points.second, label);
    return _labeled_map.getMap();
}

std::pair<int,int> Mapper::distToCoordinates(float d, float rx, float ry, float rangle, int sensor) {
    // xr,yr points in local robot axes
    float xr = 0.0;
    float yr = 0.0;

    switch(sensor) {
        case tof1:
            // Add x and y components of the sensor distance to sensor offsets
            xr = -(d*sin(M_PI/4)+TOF15_X_OFFSET);
            yr = d*cos(M_PI/4)+TOF15_Y_OFFSET;
            break;
        case tof2:
            xr = -TOF24_X_OFFSET;
            yr = d+TOF234_Y_OFFSET;
            break;
        case tof3:
            yr = d+TOF234_Y_OFFSET;
            break;
        case tof4:
            xr = TOF24_X_OFFSET;
            yr = d+TOF234_Y_OFFSET;
            break;
        case tof5:
            xr = d*sin(M_PI/4)+TOF15_X_OFFSET;
            yr = d*cos(M_PI/4)+TOF15_Y_OFFSET;
            break;
        case u1:
            xr = -(d+U123_OFFSET);
            break;
        case u2:
            yr = -(d+U123_OFFSET);
            break;
        case u3:
            xr = d+U123_OFFSET;
            break;
        default:
            ROS_ERROR("Unknown sensor type.");
            break;
    }

    // Convert xr,yr (local robot axes) to x,y (global axes)
    float robot_angle_rad = rangle*M_PI/180;
    int x = rx + xr*cos(robot_angle_rad) + yr*sin(robot_angle_rad);
    int y = ry - xr*sin(robot_angle_rad) + yr*cos(robot_angle_rad);

    return std::make_pair(x,y);
}

std::pair<int, int> Mapper::coordinateToPoints(float x, float y, int resolution) {
    float cm_per_px = 30.48/resolution;
    int point_x = ceil(x/cm_per_px);
    int point_y = ceil(y/cm_per_px);
    return std::make_pair(point_x, point_y);
}
