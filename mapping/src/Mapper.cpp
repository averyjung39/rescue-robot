#include "Mapper.hpp"
#include <math.h>

CostMap Mapper::modifyCostMap(float *dists, int robot_x, int robot_y, float robot_angle) {
    for(int i = robot_x-RADIUS; i <= robot_x+RADIUS; i++) {
        for(int j = robot_y-RADIUS; j <= robot_y+RADIUS; j++) {
            if(i < 0 || j < 0) continue;
            _cost_map[j][i] = 50;
        }
    }

    for(int i = 0; i < dists.size(); i++) {
        SensorType sensor;
        if (dists.size() == 5) {
            sensor = i;
        } else {
            sensor = 5+i;
        }
        auto points = distToPoints(dists[i], robot_x, robot_y, robot_angle, sensor);
        _cost_map[points.second][points.first] = 100;
    }
    return _cost_map;
}

std::pair<int,int> Mapper::distToPoints(float d, int rx, int ry, float rangle, int sensor) {
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
            std::cout << "Unknown sensor" << std::endl;
            break;
    }

    // Convert xr,yr (local robot axes) to x,y (global axes)
    auto robot_angle_rad = rangle*M_PI/180;
    int x = rx + ceil(xr*cos(robot_angle_rad) + yr*sin(robot_angle_rad))/CMPERPX;
    int y = ry + ceil(-xr*sin(robot_angle_rad) + yr*cos(robot_angle_rad))/CMPERPX;

    return std::make_pair(x,y);
}
