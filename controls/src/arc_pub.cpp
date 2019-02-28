#include <ros/ros.h>

#include "planning/Arc.h"
#include "topics/topics.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "arc_pub");
    ros::NodeHandle nh;

    ros::Publisher arc_pub = nh.advertise<planning::Arc>(topics::ARC_TOPIC, 1);
    planning::Arc arc_cmd;
    arc_cmd.radius = planning::Arc::STRAIGHT_LINE;
    arc_cmd.direction_is_right = false;
    arc_pub.publish(arc_cmd);

    return 0;
}