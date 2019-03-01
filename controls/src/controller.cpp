#include <ros/ros.h>

#include "controls/controller.h"
#include "planning/Arc.h"

#include "controls/wiringPi.h"
#include "controls/softPwm.h"

#define Motor1_1 20
#define Motor1_2 21
#define MotorEnable1 18

#define Motor2_1 22
#define Motor2_2 23
#define MotorEnable2 19

Controller::Controller() {
    _rpm_right = _rpm_left = 0;
    motorInit();
}

void Controller::motorInit() {
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }
    // TODO: figure out which motor is which and modify the variable names
    pinMode(Motor1_1,OUTPUT);
    pinMode(Motor1_2,OUTPUT);
    pinMode(MotorEnable1,PWM_OUTPUT);

    pinMode(Motor2_1,OUTPUT);
    pinMode(Motor2_2,OUTPUT);
    pinMode(MotorEnable2,PWM_OUTPUT);

    // Create a cycle 10 ms long made up of 100 steps
    softPwmCreate(MotorEnable1,0,MAX_PWM);
    softPwmCreate(MotorEnable2,0,MAX_PWM);
}

void Controller::actuate(const planning::Arc &arc_cmd) {
    std::pair<float, float> right_left_velocities = getVelocities(arc_cmd);

    _rpm_right = right_left_velocities.first;
    _rpm_left = right_left_velocities.second;

    ROS_INFO("Right/Left Velocities: %f, %f", _rpm_right, _rpm_left);

    bool right_is_forward = _rpm_right > 0 ? true : false;
    bool left_is_forward = _rpm_left < 0 ? true : false;

    float right_ratio = fabs(_rpm_right)/MAX_ACTUAL_RPM;
    float left_ratio = fabs(_rpm_left)/MAX_ACTUAL_RPM;

    int right_pwm = MAX_PWM*right_ratio;
    int left_pwm = MAX_PWM*left_ratio;

    actuateRightMotor(right_pwm, right_is_forward);
    actuateLeftMotor(left_pwm, left_is_forward);
}

void Controller::actuateRightMotor(int pwm, bool is_forward) {
    // TODO: figure out which setup is forward or backward
    softPwmWrite(MotorEnable1, pwm);
    if (is_forward) {
        digitalWrite(Motor1_1, HIGH);
        digitalWrite(Motor1_2, LOW);
    } else {
        digitalWrite(Motor1_1, LOW);
        digitalWrite(Motor1_2, HIGH);
    }
}

void Controller::actuateLeftMotor(int pwm, bool is_forward) {
    // TODO: figure out which setup is forward or backward
    softPwmWrite(MotorEnable2, pwm);
    if (is_forward) {
        digitalWrite(Motor2_1, LOW);
        digitalWrite(Motor2_2, HIGH);
    } else {
        digitalWrite(Motor2_1, HIGH);
        digitalWrite(Motor2_2, LOW);
    }
}

std::pair<float, float> Controller::getVelocities(const planning::Arc &arc_cmd) const {
    const float &radius = arc_cmd.radius;
    const bool &direction_is_right = arc_cmd.direction_is_right;

    std::pair<float, float> right_left_velocities = std::make_pair(_rpm_right, _rpm_left);

    if (radius == planning::Arc::STRAIGHT_LINE) {
        right_left_velocities.first = rampVelocity(MAX_ALLOWABLE_RPM, true);
        right_left_velocities.second = rampVelocity(-MAX_ALLOWABLE_RPM, false);
    } else if (radius == planning::Arc::STOP) {
        right_left_velocities.first = rampVelocity(0, true);
        right_left_velocities.second = rampVelocity(0, false);
    } else if (radius == planning::Arc::TURN_ON_SPOT) {
        int direction_sign = direction_is_right ? -1 : 1;
        right_left_velocities.first = rampVelocity(direction_sign*MAX_ALLOWABLE_RPM, true);
        right_left_velocities.second = rampVelocity(direction_sign*MAX_ALLOWABLE_RPM, false);
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
    if (target_rpm > MAX_ALLOWABLE_RPM) {
        target_rpm = MAX_ALLOWABLE_RPM;
    }
    if (fabs(target_rpm - curr_rpm) > MAX_ALLOWABLE_RPM_CHANGE) {
        return target_rpm > curr_rpm ? curr_rpm + MAX_ALLOWABLE_RPM_CHANGE : curr_rpm - MAX_ALLOWABLE_RPM_CHANGE;
    }

    return target_rpm;
}
