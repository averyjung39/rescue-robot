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
    ros::Duration timeout(0.1);
    sensors::Ultrasonic ult_data_cm;
    ult_data_cm.data.resize(1);

    while(ros::ok())
    {
        counter = 0;
        gpio28->write_gpio("1");
        ros::Duration(0.0001).sleep();
        gpio28->write_gpio("0");

        gpio31->read_gpio(inputstring);
        ros::Time begin_read_time = ros::Time::now();
        bool timed_out = false;
        while(inputstring == "0" && !timed_out)
        {
            gpio31->read_gpio(inputstring);
            start = ros::Time::now();
            timed_out = start - begin_read_time >= timeout;
        }
        begin_read_time = ros::Time::now();
        while(inputstring == "1" && !timed_out)
        {
            gpio31->read_gpio(inputstring);
            finish = ros::Time::now();
            timed_out = start - begin_read_time >= timeout;
        }
        if (timed_out) {
            ROS_WARN("Timed out!");
            continue;
        }
        ros::Duration elapsed = finish - start;
        // convert duration to seconds
	ROS_INFO("Elapsed time: %f", elapsed.toSec());
        distance = (elapsed.toSec() * 34300) / 2;
        ult_data_cm.data[0] = distance;
        ult_data_pub.publish(ult_data_cm);
    }
}
