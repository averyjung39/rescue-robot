#include <ros/ros.h>
#include <vector>

#include "sensors/TimeOfFlight.h"
#include "planning/Arc.h"
#include "topics/topics.h"

std::vector<float> tof_data;
bool new_data = false;
bool stopping = false;

void tofSensorDataCallback(const sensors::TimeOfFlight::ConstPtr& msg) {
    new_data = true;
    tof_data = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tof_test");

    ros::NodeHandle n;

    ros::Subscriber tof_data_sub = n.subscribe(topics::TOF_TOPIC, 1, tofSensorDataCallback);
    ros::Publisher arc_pub = n.advertise<planning::Arc>(topics::ARC_TOPIC, 1);

    planning::Arc arc_cmd;

    while(ros::ok()) {
        ros::spinOnce();
        if(new_data) {
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
    }

    return 0;
}
