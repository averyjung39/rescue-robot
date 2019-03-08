#include <ros/ros.h>
#include <ros/duration.h>
#include <string>

#include "sensors/Photodiode.h"
#include "sensors/GPIOClass.h"
#include "constants/topics.h"

std::vector<GPIOClass*> photodiodes;

void photodiodeInit() {
    stringstream ss;
    string pin;
    for (int i = 0; i < 5; i++) {
        // GPIO 4 to 8, from left to right
        ss << i+4;
        pin = ss.str();
        photodiodes[i] = new GPIOClass(pin);
        photodiodes[i]->export_gpio();
        photodiodes[i]->setdir_gpio("in");
    }
}

sensors::Photodiode readPhotodiodeSensors() {
    sensors::Photodiode photodiode_data;
    photodiode_data.data.resize(5);

    bool data;
    for(int i = 0; i < photodiodes.size(); i++) {
        photodiodes[i]->read_gpio(data);
        photodiode_data.data[i] = data;
    }

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

    for(int i = 0; i < photodiodes.size(); i++) {
        delete photodiodes[i];
    }
    return 0;
}
