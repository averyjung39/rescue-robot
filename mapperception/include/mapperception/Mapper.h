#ifndef MAPPER
#define MAPPER

#include <utility>
#include <vector>

#include "LabelMap.h"
#include "constants/labels.h"

// Offsets are in cm and from the center of the robot
const float TOF15_X_OFFSET = 3.048;
const float TOF15_Y_OFFSET = 0.4572+5.08;
const float TOF24_X_OFFSET = 1.905;
const float TOF234_Y_OFFSET = 5.08;
const float U123_OFFSET = 10.16;

// index of sensors in distance data vector
#define LEFT 0
#define MIDDLE 1
#define RIGHT 2

class Mapper {
public:
    Mapper() {};
    /**
     * @brief modify the label map with distance sensor data. It marks the cells as "obstacle"
     * @param dist_data: vector of distance data
     * @param robot_x, robot_y: x and y coordinates of the robot in cm
     * @param robot_angle: orientation of the robot w.r.t. the positive horizontal axis
     */
    void modifyLabelMapWithDists(std::vector<float> dist_data, bool high_sensor);
    void modifyLabelMapWithPhotodiode(std::vector<int> photodiode_data);
    void updateLabelMapWithScanningResults(bool &big_house_detected, bool &fire_detected);

    /**
     * @brief perform detection to figure out if the object in front of the robot is the fire
     * @param photodiode_data: photodiode sensor data
     * @return true if fire is detected
     */
    bool detectFire(std::vector<int> photodiode_data);
    
    /**
     * @brief perform detection to figure out if the object in front of the robot is the big house
     * @param dist_l, dist_r: top left and right tof sensor data
     * @return true if big house is detected
     */
    bool detectHouses(float dist_l, float dist_r);
    void detectMagnet(bool hall_effect_data);

    /**
     * @brief modify the label map with the label determined by perception
     * @param robot_i, robot_j: indices of the robot in the map
     * @param label: label to modify the label of the cell in front of the robot
     */
    void modifyLabelMapWithLabels(int robot_i, int robot_j, int label);

    std::pair<int, int> robotPosToPoints(float robot_x, float robot_y);

    LabelMap getLabelMap() { return _label_map; }
private:
    /**
     * @brief convert distance sensor data to coordinates in the global axis
     * @param d: distance data in cm
     * @param rx, ry: robot x and y in the map in cm
     * @param rangle: robot orientation w.r.t. to the positive horizontal axis in rad
     * @param sensor: indicator to tell which sensor the data belongs to
     * @return std::pair<int, int>: coordinates in the global axis
     */
    std::pair<float, float> distToCoordinates(float d, float rx, float ry, float rangle, int sensor);
    std::pair<int, int> coordinateToPoints(float x, float y, int resolution);
    LabelMap _label_map;
};

#endif // MAPPER
