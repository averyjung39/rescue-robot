#include <ros/ros.h>

#include "messages/Arc.h"
#include "constants/topics.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "arc_test");

    ros::NodeHandle nh;
    ros::Publisher arc_pub = nh.advertise<messages::Arc>(topics::ARC_TOPIC, 1);

    messages::Arc arc_cmd;

    int speed_r;
    int speed_l;

    // right
    if (nh.getParam("/arc_test/speed_r", speed_r)) {
        ROS_INFO("SPEED: %d", speed_r);
    } else {
        ROS_ERROR("No speed, please pass in speed_r");
        return -1;
    }
    // left
    if (nh.getParam("/arc_test/speed_l", speed_l)) {
        ROS_INFO("SPEED: %d", speed_l);
    } else {
        ROS_ERROR("No speed, please pass in speed_l");
        return -1;
    }

    while (ros::ok()) {

        if (speed_r > 0 && speed_l < 0) {
            // turn left on spot
            arc_cmd.command_type = messages::Arc::TURN_ON_SPOT;
            arc_cmd.direction_is_right = false;
        } else if (speed_r < 0 && speed_l > 0) {
            // turn right on spot
            arc_cmd.command_type = messages::Arc::TURN_ON_SPOT;
            arc_cmd.direction_is_right = true;
        } else {
            arc_cmd.command_type = messages::Arc::STRAIGHT_LINE;
            if (speed_r > 0 || speed_l > 0) {
                // forward direction
                arc_cmd.direction_is_right = true;
            } else if (speed_r < 0 || speed_l < 0) {
                // backward direction
                arc_cmd.direction_is_right = false;
            }
        }

        arc_cmd.speed_r = abs(speed_r);
        arc_cmd.speed_l = abs(speed_l);
        arc_pub.publish(arc_cmd);
    }
    return 0;
}
