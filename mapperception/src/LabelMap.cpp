#include <set>
#include <ros/ros.h>

#include "mapperception/LabelMap.h"

void LabelMap::print() const {
    std::ostringstream oss;
    oss << "\n";
    for (int i = 0; i < _size; ++i) {
        oss << i << (i < 10 ? "  : [" : (i < 100 ? " : [" : ": ["));
        for (int j = 0; j < _size; ++j) {
                int cost = _map[i][j];
                if (cost < 100) {
                    oss << "X";
                } else {
                    oss << " 0";
                }
        }
        oss << "]\n";
    }
    ROS_INFO("%s", oss.str().c_str());
}
