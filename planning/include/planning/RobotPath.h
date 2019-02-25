#ifndef ROBOT_PATH
#define ROBOT_PATH

#include <ros/ros.h>

class RobotPath {
public:
    RobotPath() {}

    const std::pair<int, int>& operator[](int i) const {
        return _path[_path.size() - i - 1];
    }
    bool empty() const {
        return _path.empty();
    }
    unsigned int size() const {
        return _path.size();
    }
    void addToFront(std::pair<int, int> indices) {
        _path.push_back(indices);
    }
    void print(int **map, const int &map_w, const int &map_h) const;

    
private:
    std::vector<std::pair<int, int> > _path;
};

#endif  // ROBOT_PATH