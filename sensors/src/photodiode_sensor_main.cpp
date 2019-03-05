#include <ros/ros.h>

#include "sensors/Photodiode.h"
#include "sensors/GPIOClass.h"
#include "topics/topics.h"

// TODO: replace pin numbers
GPIOClass* photodiode_1 = new GPIOClass("30");
GPIOClass* photodiode_2 = new GPIOClass("31");
GPIOClass* photodiode_3 = new GPIOClass("32");
GPIOClass* photodiode_4 = new GPIOClass("33");
GPIOClass* photodiode_5 = new GPIOClass("34");

void photodiodeInit() {
    photodiode_1->export_gpio();
    photodiode_1->setdir_gpio("in");

    photodiode_2->export_gpio();
    photodiode_2->setdir_gpio("in");

    photodiode_3->export_gpio();
    photodiode_3->setdir_gpio("in");

    photodiode_4->export_gpio();
    photodiode_4->setdir_gpio("in");

    photodiode_5->export_gpio();
    photodiode_5->setdir_gpio("in");
}

sensors::Photodiode readPhotodiodeSensors() {
    sensors::Photodiode photodiode_data;
    photodiode_data.data.resize(5);
    std::vector<std::string> photodiode_str(5, "");

    photodiode_1->read_gpio(photodiode_str[0]);
    photodiode_2->read_gpio(photodiode_str[1]);
    photodiode_3->read_gpio(photodiode_str[2]);
    photodiode_4->read_gpio(photodiode_str[3]);
    photodiode_5->read_gpio(photodiode_str[4]);

    for(int i = 0; i < photodiode_str.size(); i++) {
        photodiode_data.data[i] = photodiode_str[i] == "1";
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

    return 0;
}
