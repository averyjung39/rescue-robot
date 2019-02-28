#include <ros/ros.h>

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

    ros::Subscriber planning_arc_sub = nh.subscribe(topics::ARC_TOPIC, 1, planningArcCallback);
    Controller controller;
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (arc_msg) {
            controller.actuate(*arc_msg);
        }
        rate.sleep();
    }
    return 0;
}