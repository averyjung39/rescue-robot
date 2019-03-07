#include <ros/ros.h>
#include <vector>
#include <std_msgs/Bool.h>

#include "sensors/GPIOClass.h"
#include "sensors/TimeOfFlight.h"
#include "planning/Arc.h"
#include "constants/topics.h"

#ifndef CONSTRUCTION_CHECK_BUTTON_PIN
#define CONSTRUCTION_CHECK_BUTTON_PIN "45"
#endif

std::vector<float> tof_data;
bool new_data = false;
bool stopping = false;
bool turn_demo_complete = false;
bool tof_demo_complete = false;

void tofSensorDataCallback(const sensors::TimeOfFlight::ConstPtr &msg) {
    new_data = true;
    tof_data = msg->data;
}

void turnDemoCompleteCallback(const std_msgs::Bool::ConstPtr &msg) {
    turn_demo_complete = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tof_test");

    ros::NodeHandle nh;

    ros::Subscriber tof_data_sub = nh.subscribe(topics::TOF_TOPIC, 1, tofSensorDataCallback);
    ros::Publisher arc_pub = nh.advertise<planning::Arc>(topics::ARC_TOPIC, 1);

    ros::Subscriber turn_demo_complete_sub = nh.subscribe(topics::TURN_DEMO_COMPLETE_TOPIC, 1, turnDemoCompleteCallback);
    ros::Publisher tof_demo_complete_pub = nh.advertise<std_msgs::Bool>(topics::TOF_DEMO_COMPLETE_TOPIC, 1);

    planning::Arc arc_cmd;

    GPIOClass *button_gpio = new GPIOClass(CONSTRUCTION_CHECK_BUTTON_PIN);
    button_gpio->export_gpio();
    button_gpio->setdir_gpio("in");
    while (ros::ok()) {
        ros::spinOnce();

        if (!turn_demo_complete || tof_demo_complete) {
            continue;
        }

        if (new_data) {
            // Assume the first index will be populated for construction check
            if(tof_data[0] <= 5 || stopping) {
                // Speical value for stopping
                arc_cmd.radius = planning::Arc::STOP;
                arc_cmd.direction_is_right = false;
                stopping = true; // Make sure we don't start driving again after issuing a command to stop
            } else {
                // Special value for driving in a line
                arc_cmd.radius = planning::Arc::STRAIGHT_LINE;
                arc_cmd.direction_is_right = false;
            }
            arc_pub.publish(arc_cmd);
            new_data = false;
        }

        // If button is pressed, start next test
        bool input_val = false;
        if (button_gpio->read_gpio(input_val) && input_val) {
            // Wait for button release
            while (button_gpio->read_gpio(input_val) && input_val);
            std_msgs::Bool msg;
            msg.data = true;
            tof_demo_complete_pub.publish(msg);
            ROS_INFO("DONE TOF TEST");
            tof_demo_complete = true;
        }
    }

    return 0;
}
