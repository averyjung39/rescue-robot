#include <ros/ros.h>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "sensors/TimeOfFlight.h"
#include "sensors/tof.h"
#include "topics/topics.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "tof_sensor");

    ros::NodeHandle nh;

    ros::Publisher tof_data_pub = nh.advertise<sensors::TimeOfFlight>(topics::TOF_TOPIC, 1);
    int tof;
    int tof_distance;
    int model, revision;

    tof = tofInit(1, 0x29, 1); // set long range mode (up to 2m)
    if(tof != 1)
    {
        ROS_ERROR("Problem initializing ToF sensor");
        return -1; // problem - quit
    }

    ROS_INFO("VL53L0X device successfully opened.\n");
    tof = tofGetModel(&model, &revision);
    ROS_INFO("Model ID - %d\n", model);
    ROS_INFO("Revision ID - %d\n", revision);

    sensors::TimeOfFlight tof_data_cm;

    while(ros::ok()) {
        tof_distance = tofReadDistance();
        if(tof_distance < 4096) { // valid range?
            tof_data_cm.data[0] = tof_distance / 10.0;
            tof_data_pub.publish(tof_data_cm);
        }
    }

    return 0;
}
