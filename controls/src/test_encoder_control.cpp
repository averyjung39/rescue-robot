#include <ros/ros.h>

#include "constants/topics.h"
#include "external/wiringPi/wiringPi.h"
#include "planning/Arc.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_encoder_control");
    ros::NodeHandle nh("~");

    ros::Publisher arc_pub = nh.advertise<planning::Arc>(topics::ARC_TOPIC, 1);
    if (wiringPiSetup() < 0) {
        ROS_ERROR("Unable to set up wiringPi");
        return -1;
    }
    while (ros::ok()) {

    }

    return 0;
}