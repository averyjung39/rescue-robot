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
                switch(cost) {
                    case labels::FLAT_WOOD:
                        oss << " W ";
                        break;
                    case labels::SAND:
                        oss << " S ";
                        break;
                    case labels::MAGNET:
                        oss << " M ";
                        break;
                    case labels::NO_MAGNET:
                        oss << " NM";
                        break;
                    case labels::GRAVEL:
                        oss << " G ";
                        break;
                    case labels::PIT:
                        oss << " P ";
                        break;
                    case labels::FIRE:
                        oss << " F ";
                        break;
                    case labels::NO_FIRE:
                        oss << " NF";
                        break;
                    case labels::SMALL_HOUSE:
                        oss << " SH";
                        break;
                    case labels::BIG_HOUSE:
                        oss << " BH";
                        break;
                    case labels::OBJECT:
                        oss << " OB";
                        break;
                    case labels::TALL_OBJECT:
                        oss << " TO";
                        break;
                    default:
                        oss << " 0 ";
                        break;
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
