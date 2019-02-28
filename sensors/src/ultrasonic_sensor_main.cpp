#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <string>

#include "sensors/Ultrasonic.h"
#include "sensors/GPIOClass.h"
#include "topics/topics.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ultrasonic_sensor");

    ros::NodeHandle nh;
    ros::Publisher ult_data_pub = nh.advertise<sensors::Ultrasonic>(topics::ULTRASONIC_TOPIC, 1);

    GPIOClass* gpio28 = new GPIOClass("28"); // trig
    gpio28->export_gpio();
    gpio28->setdir_gpio("out");

    GPIOClass* gpio31 = new GPIOClass("31"); // echo
    gpio31->export_gpio();
    gpio31->setdir_gpio("in");

    long counter = 0;
    string inputstring;
    float distance = 0;
    ros::Time start = ros::Time::now();
    ros::Time finish = ros::Time::now();

    sensors::Ultrasonic ult_data_cm;

    while(ros::ok())
    {
        counter = 0;
        gpio28->write_gpio("1");
        ros::Duration(0.00001).sleep();
        gpio28->write_gpio("0");

        gpio31->read_gpio(inputstring);
        while(inputstring == "0")
        {
            gpio31->read_gpio(inputstring);
            start = ros::Time::now();
        }
        while(inputstring == "1")
        {
            gpio31->read_gpio(inputstring);
            finish = ros::Time::now();
        }

        ros::Duration elapsed = finish - start;
        // convert duration to seconds
        distance = (elapsed.toSec() * 34300) / 2;
        ult_data_cm.data[0] = distance;
        ult_data_pub.publish(ult_data_cm);
    }
}
