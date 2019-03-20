#ifndef MAPPER
#define MAPPER

#include <utility>
#include <vector>
#include <set>

#include "LabelMap.h"
#include "constants/labels.h"

// Offsets are in cm and from the center of the robot
// TODO: remeasure these
const float BOTTOM_TOF_Y_OFFSET = 1.905;
const float BOTTOM_TOF_X_OFFSET = 5.08;
const float TOP_TOF_FRONT_Y_OFFSET = 5.08;
const float TOP_TOF_OFFSET = 10.56;

// index of sensors in distance data vector
#define BOTTOM_LEFT 0
#define BOTTOM_RIGHT 1

#define TOP_FRONT 0
#define TOP_BACK 1
#define TOP_LEFT 2
#define TOP_RIGHT 3

class Mapper {
public:

    Mapper(int orientation);

    void setRobotPose(float robot_x, float robot_y, float robot_angle);

    /**
     * @brief modify the label map with distance sensor data. It marks the cells as "obstacle"
     * @param dist_data: vector of distance data
     * @param robot_x, robot_y: x and y coordinates of the robot in cm
     * @param robot_angle: orientation of the robot w.r.t. the positive horizontal axis
     */
    void modifyLabelMapWithDists(std::vector<float> dist_data, bool high_sensor);

    void detectFire(std::vector<int> photodiode_data);
    void detectHouses(bool big_house_detected);
    void detectMagnet(bool hall_effect_data);

    LabelMap getLabelMap() { return _label_map; }

private:
    /**
     * @brief modify the label map with the label determined by perception
     * @param robot_i, robot_j: indices of the robot in the map
     * @param label: label to modify the label of the cell in front of the robot
     */
    void modifyLabelMapWithLabels(int robot_i, int robot_j, int label);

    /**
     * @brief convert distance sensor data to coordinates in the global axis
     * @param d: distance data in cm
     * @param rx, ry: robot x and y in the map in cm
     * @param rangle: robot orientation w.r.t. to the positive horizontal axis in rad
     * @param sensor: indicator to tell which sensor the data belongs to
     * @return std::pair<int, int>: coordinates in the global axis
     */
    std::pair<float,float> distToCoordinates(float d, float rx, float ry, float rangle, int sensor, bool high_sensor);
    std::pair<int,int> coordinateToPoints(float x, float y, int resolution);
    std::pair<int,int> indicesInFront();

    LabelMap _label_map;
    // x, y, angle
    std::pair<float, float> _robot_pos;
    float _robot_angle;

    std::set<int> _found_labels;
    float _prev_ult_data;
};

#endif // MAPPER
