#include <ros/ros.h>
#include <ros/duration.h>

#include "controls/controller.h"
#include "planning/Arc.h"
#include "topics/topics.h"

#include "controls/wiringPi.h"
#include "controls/softPwm.h"

#define MotorPin1 20
#define MotorPin2 21
#define MotorEnable1 18

#define MotorPin3 22
#define MotorPin4 23
#define MotorEnable2 19

planning::Arc::ConstPtr arc_msg;

void planningArcCallback(const planning::Arc::ConstPtr &msg) {
    arc_msg = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");

    ros::NodeHandle nh;

    ros::Subscriber planning_arc_sub = nh.subscribe(topics::ARC_TOPIC, 1, planningArcCallback);
    Controller controller;
    ros::Rate rate(10);

    if(wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        return -1;
    }

    pinMode(MotorPin1,OUTPUT);
    pinMode(MotorPin2,OUTPUT);
    pinMode(MotorEnable1,PWM_OUTPUT);

    pinMode(MotorPin3,OUTPUT);
    pinMode(MotorPin4,OUTPUT);
    pinMode(MotorEnable2,PWM_OUTPUT);

    // Create a cycle 10 ms long made up of 100 steps
    softPwmCreate(MotorEnable1,0,100);
    softPwmCreate(MotorEnable2,0,100);

    while (ros::ok()) {
        ros::spinOnce();

        // keep the pulse high for 1 ms
        softPwmWrite(MotorEnable1,50);
        digitalWrite(MotorPin1,HIGH);
        digitalWrite(MotorPin2,LOW);

        ros::Duration(5).sleep();

        softPwmWrite(MotorEnable1,0);

        softPwmWrite(MotorEnable2,50);
        digitalWrite(MotorPin3,LOW);
        digitalWrite(MotorPin4,HIGH);

        ros::Duration(5).sleep();

        softPwmWrite(MotorEnable2,0);

        if (arc_msg) {
            controller.actuate(*arc_msg);
        }
        rate.sleep();
    }
    return 0;
}
