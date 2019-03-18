#include <ros/ros.h>
#include <ros/duration.h>

#include "external/wiringPi/wiringPi.h"
#include "constants/gpio_pins.h"
#include "constants/topics.h"
#include "sensors/Colour.h"

void colourSensorInit() {
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }

    pinMode(SAND, INPUT);
    pinMode(ROCK, INPUT);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "colour_sensor");

    ros::NodeHandle nh;
    ros::Publisher colour_sensor_pub = nh.advertise<sensors::Colour>(topics::COLOUR_TOPIC, 1);

    colourSensorInit();

    sensors::Colour colour_msg;
    colour_msg.data.resize(2);
    ros::Rate rate(10);

    while (ros::ok()) {
        colour_msg.data[0] = digitalRead(SAND);
        colour_msg.data[1] = digitalRead(ROCK);
        colour_sensor_pub.publish(colour_msg);
        rate.sleep();
    }

    return 0;
}
