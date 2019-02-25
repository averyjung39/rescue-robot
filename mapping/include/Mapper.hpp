const int RESOLUTION = 10;
const float CMPERPX = 30.48/RESOLUTION;
// Radius to mark searched grids. This radius means the cells that are offset by this amount
// from the current robot position in all directions will be marked as searched.
const int RADIUS = 1;
// Offsets are in cm and from the center of the robot
const float TOF15_X_OFFSET = 3.048;
const float TOF15_Y_OFFSET = 0.4572+5.08;
const float TOF24_X_OFFSET = 1.905;
const float TOF234_Y_OFFSET = 5.08;
const float U123_OFFSET = 10.16;

typedef int CostMap[6*RESOLUTION][6*RESOLUTION];

enum SensorType {
    tof1 = 0,
    tof2 = 1,
    tof3 = 2,
    tof4 = 3,
    tof5 = 4,
    u1 = 5,
    u2 = 6,
    u3 = 7
};

Class Mapper {
public:
    Mapper() {};
    CostMap modifyCostMap(float *dists, int robot_x, int robot_y, float robot_angle);
    
private:
    std::pair<int, int> distToPoints(float d, int rx, int ry, float rangle, int sensor);
    CostMap _cost_map;
};
