#ifndef OBJECTIVE_MANAGER
#define OBJECTIVE_MANAGER

#include <ros/ros.h>
#include <vector>
#include <set>

class ObjectiveManager {
public:
    ObjectiveManager();

    /**
     * @brief update _active_objectives list based on locations of the robot and the object
     * @param map: label map published my mapperception indicating where objects are
     * @param robot_i, robot_j: current robot indices
     * @return std::vector<bool>: list of active objectives
     */
    std::vector<bool> activateObjectives(int robot_i, int robot_j, std::vector< std::vector<int> > label_map);

    std::vector<bool> getActiveObjectives() { return _active_objectives; }

private:
    void turnOnIndicator(int pin);
    std::set<int> getNearLabels(int robot_i, int robot_j, std::vector<int> map);
    void setupGpio();
    std::pair<int,int> findObjectLocation(int label);

    std::vector<bool> _active_objectives;
};
#endif // OBJECTIVE_MANAGER
