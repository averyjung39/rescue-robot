#include <ros/ros.h>
#include <ros/duration.h>

#include "sensors/Photodiode.h"
#include "constants/gpio_pins.h"
#include "constants/topics.h"
#include "external/wiringPi/wiringPi.h"

void photodiodeInit() {
    pinMode(PHOTODIODE_1, INPUT);
    pinMode(PHOTODIODE_2, INPUT);
    pinMode(PHOTODIODE_3, INPUT);
    pinMode(PHOTODIODE_4, INPUT);
    pinMode(PHOTODIODE_5, INPUT);
}

sensors::Photodiode readPhotodiodeSensors() {
    sensors::Photodiode photodiode_data;
    photodiode_data.data.resize(5);

    photodiode_data.data[0] = digitalRead(PHOTODIODE_1);
    photodiode_data.data[1] = digitalRead(PHOTODIODE_2);
    photodiode_data.data[2] = digitalRead(PHOTODIODE_3);
    photodiode_data.data[3] = digitalRead(PHOTODIODE_4);
    photodiode_data.data[4] = digitalRead(PHOTODIODE_5);

    return photodiode_data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "photodiode_sensor");

    ros::NodeHandle nh;
    ros::Publisher photodiode_data_pub = nh.advertise<sensors::Photodiode>(topics::PHOTODIODE_TOPIC, 1);

    photodiodeInit();

    sensors::Photodiode photodiode_data;
    ros::Rate rate(10);

    while (ros::ok()) {
        photodiode_data = readPhotodiodeSensors();
        photodiode_data_pub.publish(photodiode_data);
        rate.sleep();
    }

    return 0;
}
