#include <ros/ros.h>
#include <ros/duration.h>

#include "controls/controller.h"
#include "planning/Arc.h"
#include "topics/topics.h"

planning::Arc::ConstPtr arc_msg;

void planningArcCallback(const planning::Arc::ConstPtr &msg) {
    arc_msg = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");

    ros::NodeHandle nh;

    int speed_r;
    int speed_l;
    // 1 for both, 2 for right, 3 for left;
    // int motor_to_run;

    // right
    if (nh.getParam("/controller/speed_r", speed_r)) {
        ROS_INFO("SPEED: %d", speed_r);
    } else {
        ROS_ERROR("No speed, please pass in speed_r");
        return -1;
    }
    // left
    if (nh.getParam("/controller/speed_l", speed_l)) {
        ROS_INFO("SPEED: %d", speed_l);
    } else {
        ROS_ERROR("No speed, please pass in speed_l");
        return -1;
    }

/*
    if (nh.getParam("/controller/motor_to_run", motor_to_run)) {
        ROS_INFO("MOTOR TO RUN: %d", motor_to_run);
    } else {
        ROS_ERROR("No motor selected, please select motor");
        return -1;
    }

    if (motor_to_run < 1 || motor_to_run > 3) {
        ROS_ERROR("Invalid motor to run, please select from 1-3");
        return -1;
    }
*/

    ros::Subscriber planning_arc_sub = nh.subscribe(topics::ARC_TOPIC, 1, planningArcCallback);
    Controller controller;
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        if (arc_msg) {
            controller.actuate(*arc_msg, speed_r, speed_l); //motor_to_run);
        }
        rate.sleep();
    }
    return 0;
}
