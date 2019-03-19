#include <ros/ros.h>

#include "controls/controller.h"
#include "messages/Arc.h"
#include "constants/gpio_pins.h"
#include "external/wiringPi/wiringPi.h"
#include "external/wiringPi/softPwm.h"

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

void Controller::actuate(const messages::Arc &arc_cmd) {
    std::pair<float, float> right_left_velocities = getVelocities(arc_cmd);

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

std::pair<float, float> Controller::getVelocities(const messages::Arc &arc_cmd) const {
    const float &command_type = arc_cmd.command_type;
    const bool &direction_is_right = arc_cmd.direction_is_right;
    const float &speed_r = arc_cmd.speed_r;
    const float &speed_l = arc_cmd.speed_l;

    std::pair<float, float> right_left_velocities = std::make_pair(_rpm_right, _rpm_left);

    if (command_type == messages::Arc::STRAIGHT_LINE) {
        // right is forward
        int direction_sign = direction_is_right ? 1 : -1;
        right_left_velocities.first = rampVelocity(direction_sign*speed_r, true);
        right_left_velocities.second = rampVelocity(-direction_sign*speed_l, false);
    } else if (command_type == messages::Arc::STOP) {
        right_left_velocities.first = rampVelocity(0, true);
        right_left_velocities.second = rampVelocity(0, false);
    } else if (command_type == messages::Arc::TURN_ON_SPOT) {
        int direction_sign = direction_is_right ? -1 : 1;
        right_left_velocities.first = rampVelocity(direction_sign*speed_r, true);
        right_left_velocities.second = rampVelocity(direction_sign*speed_l, false);
    } else {
        ROS_ERROR("UNHANDLED CONTROLS CASE: command_type=%f, direction=%d", command_type, direction_is_right);
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
