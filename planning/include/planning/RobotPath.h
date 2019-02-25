#ifndef ROBOT_PATH
#define ROBOT_PATH

#include <ros/ros.h>

class RobotPath {
public:
    RobotPath() {}

    /**
     * @brief  Accessor for a map cell at a given index
     * @param  i: Desired index (0 = start of path, size-1 = end of path)
     * @return (row,col) indices at the desired path index
     */
    const std::pair<int, int>& operator[](int i) const {
        // The way that _path is implemented, start of the path is the last index
        // The end of the path is _path[0]
        return _path[_path.size() - i - 1];
    }
    bool empty() const {
        return _path.empty();
    }
    unsigned int size() const {
        return _path.size();
    }

    /**
     * @brief Adds a new map cell to the front path
     * @param indices: The (row,col) pair to add to the path
     */
    void addToFront(std::pair<int, int> indices) {
        _path.push_back(indices);
    }

    /**
     * @brief Prints path, start and goal position, and map to console
     * @param map: The costmap
     * @param map_w, map_h: Map width and height (number of rows and cols)
     */
    void print(int **map, const int &map_w, const int &map_h) const;

    
private:
    std::vector<std::pair<int, int> > _path;
};

#endif  // ROBOT_PATH