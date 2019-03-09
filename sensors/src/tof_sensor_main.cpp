#include <ros/ros.h>
#include <ros/duration.h>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "sensors/TimeOfFlight.h"
#include "sensors/tof.h"
#include "constants/topics.h"

/*
1. Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
2. Keep sensor #1 awake by keeping XSHUT pin high
3. Put all other sensors into shutdown by pulling XSHUT pins low
4. Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Goingwith 0x30 to 0x3F is probably OK.
5. Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
6. Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
Don't forget to remove the protective plastic cover from the sensor before using! 
7. Repeat for each sensor, turning each one on, setting a unique address.Note you must do this every time you turn on the power, the addresses are not permanent*/

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
    tof_data_cm.data.resize(1);
    ros::Rate rate(10);

    while (ros::ok()) {
        tof_distance = tofReadDistance();
        if(tof_distance < 4096) { // valid range?
            tof_data_cm.data[0] = tof_distance / 10.0;
            tof_data_pub.publish(tof_data_cm);
        }
        rate.sleep();
    }

    return 0;
}
