#ifndef OBJECTIVE_MANAGER
#define OBJECTIVE_MANAGER

#include <ros/ros.h>
#include <vector>

#include "mapperception/ObjectLocation.h"

class ObjectiveManager {
public:
    ObjectiveManager(ros::NodeHandle& nh, const char* service_name);

    /**
     * @brief update _active_objectives list based on locations of the robot and the object
     * @param map: label map published my mapperception indicating where objects are
     * @param robot_i, robot_j: current robot indices
     * @return std::vector<bool>: list of active objectives
     */
    std::vector<bool> activateObjectives(int robot_i, int robot_j);

    std::vector<bool> getActiveObjectives() { return _active_objectives; }

private:
    void turnFanOnOff(bool on);
    void turnOnIndicator(int pin);
    void setupGpio();
    std::pair< std::vector<int>, std::vector<int> > findObjectLocation(int label);

    ros::ServiceClient _obj_loc_client;
    mapperception::ObjectLocation _srv;

    std::vector<bool> _active_objectives;
};
#endif // OBJECTIVE_MANAGER
