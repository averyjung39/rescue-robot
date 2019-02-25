#include "LabeledMap.hpp"
#include "TerrainMap.hpp"

// Radius to mark searched grids. This radius means the cells that are offset by this amount
// from the current robot position in all directions will be marked as searched.
const int RADIUS = 1;
// Offsets are in cm and from the center of the robot
const float TOF15_X_OFFSET = 3.048;
const float TOF15_Y_OFFSET = 0.4572+5.08;
const float TOF24_X_OFFSET = 1.905;
const float TOF234_Y_OFFSET = 5.08;
const float U123_OFFSET = 10.16;

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
    LabeledMap modifyLabeledMap(float *dists, float robot_x, float robot_y, float robot_angle);
    TerrainMap modifyTerrainMap(float x, float y, Terrain terrain);
private:
    std::pair<int, int> distToCoordinates(float d, float rx, float ry, float rangle, int sensor);
    std::pair<int, int> coordinateToPoints(float x, float y, int resolution);
    LabeledMap _labeled_map;
    TerrainMap _terrain_map;
};
