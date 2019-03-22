#include <ros/ros.h>
#include <ros/duration.h>

#include "controls/controller.h"
#include "messages/Arc.h"
#include "constants/topics.h"
#include "std_msgs/Bool.h"

messages::Arc::ConstPtr arc_msg;

void planningArcCallback(const messages::Arc::ConstPtr &msg) {
    arc_msg = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");

    ros::NodeHandle nh;

    ros::Subscriber planning_arc_sub = nh.subscribe(topics::ARC_TOPIC, 1, planningArcCallback);
    ros::Publisher controller_done_pub = nh.advertise<std_msgs::Bool>(topics::CONTROLLER_DONE, 1);
    Controller controller;
    ros::Rate rate(10);
    std_msgs::Bool controller_done_msg;

    float tile_time; // Time to travel one tile
    float right_turn_time;
    float left_turn_time;
    nh.param<float>("/tile_time", tile_time, 2.05);
    nh.param<float>("/right_turn_time", right_turn_time, 2.1);
    nh.param<float>("/left_turn_time", left_turn_time, 1.8);


    while (ros::ok()) {
        ros::spinOnce();
        controller_done_msg.data =false;
        controller_done_pub.publish(controller_done_msg);
        if (arc_msg) {
            float time = 0;
            if (arc_msg->command_type == messages::Arc::STRAIGHT_LINE) {
                time = tile_time*arc_msg->num_tiles;
            } else if (arc_msg->command_type == messages::Arc::TURN_ON_SPOT) {
                time = arc_msg->direction_is_right ? right_turn_time : left_turn_time;
            }
            controller.actuate(*arc_msg, time);

        }
        controller_done_msg.data = true;
        controller_done_pub.publish(controller_done_msg);
        rate.sleep();
    }
    return 0;
}
