#ifndef OBJECTIVE_MANAGER
#define OBJECTIVE_MANAGER

#include <vector>
#include "mapperception/Map.h"

class ObjectiveManager {
public:
    ObjectiveManager();

    /**
     * @brief update _active_objectives list based on locations of the robot and the object
     * @param map: label map published my mapperception indicating where objects are
     * @param robot_i, robot_j: current robot indices
     */
    void activateObjectives(mapperception::Map label_map, int robot_i, int robot_j);

    std::vector<Objectives> getActiveObjectives() { return _active_objectives; }
private:
    void turnFanOnOff(bool on);
    void turnOnIndicator(int pin);
    void setupGpio();
    std::pair<int,int> findObjectLocation(std::vector< std::vector<int> > label_map, int label);

    std::vector<bool> _active_objectives;
}
#endif // OBJECTIVE_MANAGER
