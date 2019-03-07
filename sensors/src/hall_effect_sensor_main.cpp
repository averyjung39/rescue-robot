#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>

#include "external/wiringPi.h"
#include "constants/gpio_pins.h"
#include "constants/topics.h"

void hallEffectInit() {
    pinMode(HALL_EFFECT_1_H, INPUT);
    pinMode(HALL_EFFECT_1_L, INPUT);
    pinMode(HALL_EFFECT_2_H, INPUT);
    pinMode(HALL_EFFECT_2_L, INPUT);
}

bool checkForMagnet() {
    if (digitalRead(HALL_EFFECT_1_H) == HIGH ||
        digitalRead(HALL_EFFECT_1_L) == LOW ||
        digitalRead(HALL_EFFECT_2_H) == HIGH ||
        digitalRead(HALL_EFFECT_2_L) == LOW) {
        return true;
    }
    return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hall_effect_sensor");

    ros::NodeHandle nh;
    ros::Publisher hall_effect_data_pub = nh.advertise<std_msgs::Bool>(topics::HALL_EFFECT_TOPIC, 1);

    hallEffectInit();

    std_msgs::Bool hall_effect_msg;
    ros::Rate rate(10);

    while (ros::ok()) {
        hall_effect_msg.data = checkForMagnet();
        hall_effect_data_pub.publish(hall_effect_msg);
        rate.sleep();
    }

    return 0;
}
