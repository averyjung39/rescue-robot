#include <ros/ros.h>
#include <ros/duration.h>

#include "controls/controller.h"
#include "planning/ControlCommand.h"
#include "constants/topics.h"

planning::ControlCommand::ConstPtr control_cmd_msg;

void planningControlCmdCallback(const planning::ControlCommand::ConstPtr &msg) {
    control_cmd_msg = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");

    ros::NodeHandle nh;

    ros::Subscriber control_cmd_sub = nh.subscribe(topics::CONTROL_COMMAND_TOPIC, 1, planningControlCmdCallback);
    Controller controller;
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        if (control_cmd_msg) {
            controller.actuate(*control_cmd_msg);
        }
        rate.sleep();
    }
    return 0;
}
