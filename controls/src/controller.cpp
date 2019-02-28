#include <ros/ros.h>

#include "controls/controller.h"
#include "planning/Arc.h"

Controller::Controller() {
    _rpm_right = _rpm_left = 0;
}

void Controller::actuate(const planning::Arc &arc_cmd) {
    std::pair<float, float> right_left_velocities = getVelocities(arc_cmd);

    _rpm_right = right_left_velocities.first;
    _rpm_left = right_left_velocities.second;

    ROS_INFO("Right/Left Velocities: %f, %f", _rpm_right, _rpm_left);

}

std::pair<float, float> Controller::getVelocities(const planning::Arc &arc_cmd) const {
    const float &radius = arc_cmd.radius;
    const bool &direction_is_right = arc_cmd.direction_is_right;

    std::pair<float, float> right_left_velocities = std::make_pair(_rpm_right, _rpm_left);

    if (radius == planning::Arc::STRAIGHT_LINE) {
        right_left_velocities.first = rampVelocity(MAX_VELOCITY_RPM, true);
        right_left_velocities.second = rampVelocity(-MAX_VELOCITY_RPM, false);
    } else if (radius == planning::Arc::STOP) {
        right_left_velocities.first = rampVelocity(0, true);
        right_left_velocities.second = rampVelocity(0, false);
    } else if (radius == planning::Arc::TURN_ON_SPOT) {
        int direction_sign = direction_is_right ? -1 : 1;
        right_left_velocities.first = rampVelocity(direction_sign*MAX_VELOCITY_RPM, true);
        right_left_velocities.second = rampVelocity(direction_sign*MAX_VELOCITY_RPM, false);
    } else {
        ROS_ERROR("UNHANDLED CONTROLS CASE: radius=%f, direction=%d", radius, direction_is_right);
    }

    return right_left_velocities;
}

float Controller::rampVelocity(float target_rpm, const bool &is_right_motor) const {
    float curr_rpm;
    if (is_right_motor) {
        curr_rpm = _rpm_right;
    } else {
        curr_rpm = _rpm_left;
    }

    // Limit max rpm
    if (target_rpm > MAX_VELOCITY_RPM) {
        target_rpm = MAX_VELOCITY_RPM;
    }
    if (fabs(target_rpm - curr_rpm) > MAX_VELOCITY_RPM_CHANGE) {
        return target_rpm > curr_rpm ? curr_rpm + MAX_VELOCITY_RPM_CHANGE : curr_rpm - MAX_VELOCITY_RPM_CHANGE;
    }

    return target_rpm;
}
