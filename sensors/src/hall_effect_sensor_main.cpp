#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>

#include "external/wiringPi/wiringPi.h"
#include "constants/gpio_pins.h"
#include "constants/topics.h"

void hallEffectInit() {
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }

    pinMode(HALL_EFFECT_1, INPUT);
    pinMode(HALL_EFFECT_2, INPUT);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hall_effect_sensor");

    ros::NodeHandle nh;
    ros::Publisher hall_effect_data_pub = nh.advertise<std_msgs::Bool>(topics::HALL_EFFECT_TOPIC, 1);

    hallEffectInit();

    std_msgs::Bool hall_effect_msg;
    std::vector<bool> read_data(2,true);
    ros::Rate rate(10);

    while (ros::ok()) {
        read_data[0] = true;
        read_data[1] = true;
        for (int i = 0; i < 5; i++) {
            read_data[0] = read_data[0] & digitalRead(HALL_EFFECT_1);
            read_data[1] = read_data[1] & digitalRead(HALL_EFFECT_2);
        }
        hall_effect_msg.data = (read_data[0] || read_data[1]);
        hall_effect_data_pub.publish(hall_effect_msg);
        rate.sleep();
    }

    return 0;
}
