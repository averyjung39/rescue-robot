#ifndef CONTROLLER
#define CONTROLLER

#include "messages/Arc.h"

class Controller {
public:
    Controller();

    void actuate(const messages::Arc &arc_cmd);
private:
    static const float MAX_ALLOWABLE_RPM_CHANGE = 5;
    static const float MAX_ALLOWABLE_RPM = 280;
    static const float MAX_ACTUAL_RPM = 280;
    static const float MAX_PWM = 100;

    std::pair<float, float> getVelocities(const messages::Arc &arc_cmd) const;
    float rampVelocity(float target_rpm, const bool &is_right_motor) const;

    void motorInit();
    void actuateRightMotor(int pwm, bool is_forward) const;
    void actuateLeftMotor(int pwm, bool is_forward) const;

    float _rpm_right;
    float _rpm_left;
};

#endif  // CONTROLLER
