#include <ros/ros.h>
#include <vector>

#include "sensors/Ultrasonic.h"
#include "planning/Arc.h"
#include "topics/topics.h"

std::vector<float> data;
bool new_data = false;
bool stopping = false;

void ultrasonicSensorDataCallback(const sensors::Ultrasonic::ConstPtr& msg) {
    new_data = true;
    data = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ultrasonic_test");

    ros::NodeHandle nh;

    ros::Subscriber tof_data_sub = nh.subscribe(topics::ULTRASONIC_TOPIC, 1, ultrasonicSensorDataCallback);
    ros::Publisher arc_pub = nh.advertise<planning::Arc>(topics::ARC_TOPIC, 1);

    planning::Arc arc_cmd;

    while (ros::ok()) {
        ros::spinOnce();
        // Assume back ultrasonic sensor is mounted
        // Start close to a wall and drive away from the wall until
        // we are 50 cm from the wall
        if (new_data) {
            // Assume the first index will be populated for construction check
            if (data[0] > 50 || stopping) {
                // Speical value for stopping
                arc_cmd.radius = -2;
                arc_cmd.direction_is_right = false;
                stopping = true; // Make sure we don't start driving again after issuing a command to stop
            } else {
                // Special value for driving in a line
                arc_cmd.radius = -1;
                arc_cmd.direction_is_right = false;
            }
            arc_pub.publish(arc_cmd);
            new_data = false;
        }
    }

    return 0;
}
