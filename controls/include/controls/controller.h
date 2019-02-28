#ifndef CONTROLLER
#define CONTROLLER

#include "planning/Arc.h"

class Controller {
public:
    Controller();

    void actuate(const planning::Arc &arc_cmd);
private:
    static const float MAX_VELOCITY_RPM_CHANGE = 5;
    static const float MAX_VELOCITY_RPM = 150;

    std::pair<float, float> getVelocities(const planning::Arc &arc_cmd) const;
    float rampVelocity(float target_rpm, const bool &is_right_motor) const;

    float _rpm_right;
    float _rpm_left;
};

#endif  // CONTROLLER
