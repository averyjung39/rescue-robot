#include <ros/ros.h>

#include "controls/controller.h"
#include "planning/Arc.h"

#include "controls/wiringPi.h"
#include "controls/softPwm.h"

#define MOTOR_RIGHT_1 20
#define MOTOR_RIGHT_2 21
#define MOTOR_ENABLE_1 18

#define MOTOR_LEFT_1 22
#define MOTOR_LEFT_2 23
#define MOTOR_ENABLE_2 19

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
    pinMode(MOTOR_RIGHT_1,OUTPUT);
    pinMode(MOTOR_RIGHT_2,OUTPUT);
    pinMode(MOTOR_ENABLE_1,PWM_OUTPUT);

    pinMode(MOTOR_LEFT_1,OUTPUT);
    pinMode(MOTOR_LEFT_2,OUTPUT);
    pinMode(MOTOR_ENABLE_2,PWM_OUTPUT);

    // Create a cycle 10 ms long made up of 100 steps
    softPwmCreate(MOTOR_ENABLE_1,0,MAX_PWM);
    softPwmCreate(MOTOR_ENABLE_2,0,MAX_PWM);
}

void Controller::actuate(const planning::Arc &arc_cmd, int speed_r, int speed_l) {// int motor_to_run) {
    std::pair<float, float> right_left_velocities = getVelocities(arc_cmd, speed_r, speed_l);

    _rpm_right = right_left_velocities.first;
    _rpm_left = right_left_velocities.second;

    ROS_INFO("Right/Left Velocities: %f, %f", _rpm_right, _rpm_left);

    bool right_is_forward = _rpm_right > 0;
    bool left_is_forward = _rpm_left < 0;

    float right_ratio = fabs(_rpm_right)/MAX_ACTUAL_RPM;
    float left_ratio = fabs(_rpm_left)/MAX_ACTUAL_RPM;

    int right_pwm = MAX_PWM*right_ratio;
    int left_pwm = MAX_PWM*left_ratio;

    actuateRightMotor(right_pwm, right_is_forward);
    actuateLeftMotor(left_pwm, left_is_forward);

    /*if (motor_to_run == 2) {
        actuateRightMotor(right_pwm, right_is_forward);
    } else if (motor_to_run == 3) {
        actuateLeftMotor(left_pwm, left_is_forward);
    } else {
        actuateRightMotor(right_pwm, right_is_forward);
        actuateLeftMotor(left_pwm, left_is_forward);
    }*/
}

void Controller::actuateRightMotor(int pwm, bool is_forward) const {
    // TODO: figure out which setup is forward or backward
    softPwmWrite(MOTOR_ENABLE_1, pwm);
    if (is_forward) {
        digitalWrite(MOTOR_RIGHT_1, HIGH);
        digitalWrite(MOTOR_RIGHT_2, LOW);
    } else {
        digitalWrite(MOTOR_RIGHT_1, LOW);
        digitalWrite(MOTOR_RIGHT_2, HIGH);
    }
}

void Controller::actuateLeftMotor(int pwm, bool is_forward) const {
    // TODO: figure out which setup is forward or backward
    softPwmWrite(MOTOR_ENABLE_2, pwm);
    if (is_forward) {
        digitalWrite(MOTOR_LEFT_1, LOW);
        digitalWrite(MOTOR_LEFT_2, HIGH);
    } else {
        digitalWrite(MOTOR_LEFT_1, HIGH);
        digitalWrite(MOTOR_LEFT_2, LOW);
    }
}

std::pair<float, float> Controller::getVelocities(const planning::Arc &arc_cmd, float speed_r, float speed_l) const {
    const float &radius = arc_cmd.radius;
    const bool &direction_is_right = arc_cmd.direction_is_right;

    std::pair<float, float> right_left_velocities = std::make_pair(_rpm_right, _rpm_left);

    if (radius == planning::Arc::STRAIGHT_LINE) {
        right_left_velocities.first = rampVelocity(speed_r, true);
        right_left_velocities.second = rampVelocity(-speed_l, false);
    } else if (radius == planning::Arc::STOP) {
        right_left_velocities.first = rampVelocity(0, true);
        right_left_velocities.second = rampVelocity(0, false);
    } else if (radius == planning::Arc::TURN_ON_SPOT) {
        int direction_sign = direction_is_right ? -1 : 1;
        right_left_velocities.first = rampVelocity(direction_sign*speed_r, true);
        right_left_velocities.second = rampVelocity(direction_sign*speed_l, false);
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
    if (target_rpm < -MAX_ALLOWABLE_RPM) {
        target_rpm = -MAX_ALLOWABLE_RPM;
    }
    if (fabs(target_rpm - curr_rpm) > MAX_ALLOWABLE_RPM_CHANGE) {
        return target_rpm > curr_rpm ? curr_rpm + MAX_ALLOWABLE_RPM_CHANGE : curr_rpm - MAX_ALLOWABLE_RPM_CHANGE;
    }

    return target_rpm;
}
