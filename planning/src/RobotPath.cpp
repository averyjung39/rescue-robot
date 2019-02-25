#include <set>

#include "planning/RobotPath.h"

void RobotPath::print(int **map, const int &map_w, const int &map_h) const {
    std::ostringstream oss;
    std::set<std::pair<int, int> > path_map(_path.begin(), _path.end());
    std::pair<int, int> start_indices = (*this)[0];
    std::pair<int, int> end_indices = (*this)[size()-1];
    oss << "\n";
    for (int i = 0; i < map_h; ++i) {
        oss << i << (i < 10 ? "  : [" : (i < 100 ? " : [" : ": ["));
        for (int j = 0; j < map_w; ++j) {
            std::pair<int, int> indices = std::make_pair(i, j);
            if (indices == start_indices) {
                oss << " S";
            } else if (indices == end_indices) {
                oss << " G";
            } else if (path_map.count(indices)) {
                oss << " *";
            } else {
                int cost = map[i][j];
                if (cost < 100) {
                    oss << "  ";
                } else {
                    oss << " X";
                }
            }
        }
        oss << "]\n";
    }
    ROS_INFO("%s", oss.str().c_str());
}