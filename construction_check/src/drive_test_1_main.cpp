#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>

#include "sensors/GPIOClass.h"
#include "planning/Arc.h"
#include "constants/topics.h"

#ifndef CONSTRUCTION_CHECK_BUTTON_PIN
#define CONSTRUCTION_CHECK_BUTTON_PIN "45"
#endif

bool drive_demo_complete = false;

int main(int argc, char **argv) {
    ros::init(argc, argv, "drive_forward_backward_test");

    ros::NodeHandle nh;
    ros::Publisher arc_pub = nh.advertise<planning::Arc>(topics::ARC_TOPIC, 1);
    ros::Publisher drive_demo_complete_pub = nh.advertise<std_msgs::Bool>(topics::DRIVE_DEMO_COMPLETE_TOPIC, 1);

    ros::Duration run_duration(5);
    ros::Duration stop_duration(1);
    bool stop = false;

    planning::Arc arc_cmd;
    arc_cmd.radius = planning::Arc::STRAIGHT_LINE;
    // direction right is for going forward
    arc_cmd.direction_is_right = true;
    arc_cmd.speed_r = 70;
    arc_cmd.speed_l = 70;

    GPIOClass *button_gpio = new GPIOClass(CONSTRUCTION_CHECK_BUTTON_PIN);
    button_gpio->export_gpio();
    button_gpio->setdir_gpio("in");


    // Wait for button press
    bool val = false;
    while (!val) {
        button_gpio->read_gpio(val);
    }
    ROS_INFO("Detected button press!");
    // Wait for button release
    while (val) {
        button_gpio->read_gpio(val);
    }
    ROS_INFO("Beginning Construction Check Demo");
    while (ros::ok()) {
        if (drive_demo_complete) {
            continue;
        }

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

        // If button is pressed, start next test
        bool input_val = false;
        if (button_gpio->read_gpio(input_val) && input_val) {
            // Wait for button release
            while (button_gpio->read_gpio(input_val) && input_val);
            std_msgs::Bool msg;
            msg.data = true;
            drive_demo_complete_pub.publish(msg);
            ROS_INFO("DONE DRIVE TEST");
            drive_demo_complete = true;
        }
    }
    return 0;
}
