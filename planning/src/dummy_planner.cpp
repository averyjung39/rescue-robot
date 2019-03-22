#include <ros/ros.h>
#include "constants/topics.h"
#include "external/wiringPi/wiringPi.h"
#include "messages/Arc.h"

bool controller_done;

void controllerDoneCallback(const std_msgs::Bool::ConstPtr &msg) {
    controller_done = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dummy_planner");
    ros::NodeHandle nh;
    ros::Publisher arc_pub = nh.advertise<messages::Arc>(topics::ARC_TOPIC, 1);
    ros::Subscriber controller_done_sub = nh.subscribe(topics::CONTROLLER_DONE, 1, controllerDoneCallback);
    messages::Arc arc_cmd;

    int command_type;
    bool direction_is_right;
    int num_tiles;
    int speed_r, speed_l;
    float time;
    nh.param<int>("/dummy_planner/command_type", command_type, messages::Arc::STRAIGHT_LINE);
    nh.param<bool>("/dummy_planner/direction_is_right", direction_is_right, true);
    nh.param<int>("/dummy_planner/num_tiles", num_tiles, 1);
    nh.param<int>("/dummy_planner/speed_r", speed_r, 120);
    nh.param<int>("/dummy_planner/speed_l", speed_l, 110);
    if (command_type == messages::Arc::STRAIGHT_LINE) {
        nh.param<float>("/tile_time", time, 2.05);
    } else if (command_type == messages::Arc::TURN_ON_SPOT) {
        if (direction_is_right) {
            nh.param<float>("/right_turn_time", time, 2.1);
        } else {
            nh.param<float>("/left_turn_time", time, 1.8);
        }
    } else {
        ROS_ERROR("Invalid command_type.");
        return -1;
    }

    if (wiringPiSetup() < 0) {
        ROS_ERROR("Unable to set up wiringPi");
        return -1;
    }

    arc_cmd.direction_is_right = direction_is_right;
    arc_cmd.num_tiles = num_tiles;
    arc_cmd.speed_r = speed_r;
    arc_cmd.speed_l = speed_l;
    
    pinMode(45, INPUT);
    
    // float start_time = ros::Time::now().toSec();
    while (ros::ok()) {
        while(!digitalRead(45));
        while(digitalRead(45));
        controller_done = false;
        arc_cmd.command_type = command_type;
        arc_pub.publish(arc_cmd);
        while (!controller_done) {
            ros::spinOnce();
        }
        arc_cmd.command_type = messages::Arc::STOP;
        arc_pub.publish(arc_cmd);
        while (!controller_done) {
            ros::spinOnce();
        }
        ROS_INFO("Done.");
    }
    return 0;
}