#include <ros/ros.h>
#include <ros/duration.h>

#include "controls/controller.h"
#include "planning/Arc.h"
#include "topics/topics.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "drive_forward_backward_test");

    ros::NodeHandle nh;
    ros::Publisher arc_pub = nh.advertise<planning::Arc>(topics::ARC_TOPIC, 1);

    Controller controller;

    ros::Duration run_duration(5);
    ros::Duration stop_duration(1);
    bool stop = false;

    planning::Arc arc_cmd;
    arc_cmd.radius = planning::Arc::STRAIGHT_LINE;
    // direction right is for going forward 
    arc_cmd.direction_is_right = true;

    while (ros::ok()) {
        if (!stop) {
            arc_pub.publish(arc_cmd);
            run_duration.sleep();

            arc_cmd.radius = planning::Arc::STOP;
            arc_pub.publish(arc_cmd);
            stop_duration.sleep();

            arc_cmd.radius = planning::Arc::STRAIGHT_LINE;
            arc_cmd.direction_is_right = false;
            arc_pub.publish(arc_cmd);
            run_duration.sleep();

            arc_cmd.radius = planning::Arc::STOP;
            arc_pub.publish(arc_cmd);
            stop = true;
        }
    }
    return 0;
}
