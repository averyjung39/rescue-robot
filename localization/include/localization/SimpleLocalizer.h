#ifndef SIMPLE_LOCALIZER
#define SIMPLE_LOCALIZER

#include <vector>

#include "localization/Pose.h"
#include "messages/Arc.h"

class SimpleLocalizer {
public:
    SimpleLocalizer();
    localization::Pose getPoseEstimate(
        const messages::Arc &arc_msg,
        const float &imu_yaw,
        const std::vector<float> &high_distance_data);

private:
    static const int FRONT_INDEX = 0;
    static const int BACK_INDEX = 1;
    static const float MAX_STRAIGHT_LINE_DISTANCE_DEVIATION_CM = 10; // TODO tune this 
    static const float FRONT_HIGH_DISTANCE_SENSOR_OFFSET_CM = 15; // TODO measure this
    static const float BACK_HIGH_DISTANCE_SENSOR_OFFSET_CM = 15; // TODO measure this

    localization::Pose _current_pose;
    int _prev_control_command;
    float _imu_yaw_deg;
    float _front_distance_cm;
    float _back_distance_cm;
    float _prev_front_distance_cm;
    float _prev_back_distance_cm;
    int _nominal_theta_deg; // Closest angle to yaw that's a multiple of 90 (when driving straight)

    bool isValidDistanceReading(const float &distance,
        const float &prev_distance) const;
};

#endif  // SIMPLE_LOCALIZER