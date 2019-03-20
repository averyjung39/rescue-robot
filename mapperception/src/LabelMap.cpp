#include <set>
#include <ros/ros.h>

#include "mapperception/LabelMap.h"
#include "constants/labels.h"

void LabelMap::print() const {
    std::ostringstream oss;
    oss << "\n";
    for (int i = 0; i < _size; ++i) {
        oss << i << (i < 10 ? "  : [" : (i < 100 ? " : [" : ": ["));
        for (int j = 0; j < _size; ++j) {
                int cost = _map[i][j];
                if (cost != 0) {
                    oss << " X";
                } else {
                    oss << " 0";
                }
        }
        oss << "]\n";
    }
    ROS_INFO("%s", oss.str().c_str());
}

int LabelMap::queryMap(int row, int col) {
    if (row > 5 || col > 5 || row < 0 || col < 0) {
        ROS_WARN("Out of bound {%d, %d}", row, col);
        return labels::INVALID_LABEL;
    }
    return _map[row][col];
}
