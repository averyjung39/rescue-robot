#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include "sensors/Ultrasonic.h"
#include "constants/gpio_pins.h"
#include "constants/topics.h"
#include "external/wiringPi/wiringPi.h"

void ultrasonicInit(){
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }

    pinMode(ULTRASONIC_F_TRIG, OUTPUT);
    pinMode(ULTRASONIC_F_ECHO, INPUT);
}

/**
 * @param trig_pin trigger pin number of ultrasonic sensor
 * @param echo_pin echo pin number of ultrasonic sensor
 * @param elapsed variable to store elapsed time for calculating distance
 * @return bool true will be returned if the sensor data was read without being timed out
 */
bool readUltrasonic(int trig_pin, int echo_pin, ros::Duration &elapsed) {
    // Set timeout to 100ms
    ros::Duration timeout(0.1);
    ros::Time start = ros::Time::now();
    ros::Time finish = ros::Time::now();
    bool timed_out = false;

    digitalWrite(trig_pin, HIGH);
    ros::Duration(0.0001).sleep();
    digitalWrite(trig_pin, LOW);

    ros::Time begin_read_time = ros::Time::now();
    while (digitalRead(echo_pin) == LOW && !timed_out) {
        start = ros::Time::now();
        timed_out = start - begin_read_time >= timeout;
    }
    begin_read_time = ros::Time::now();
    while (digitalRead(echo_pin) == HIGH && !timed_out) {
        finish = ros::Time::now();
        timed_out = finish - begin_read_time >= timeout;
    }
    if (timed_out) {
        return false;
    }

    elapsed = finish - start;
    return true;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "ultrasonic_sensor");

    ros::NodeHandle nh;
    ros::Publisher ult_data_pub = nh.advertise<sensors::Ultrasonic>(topics::ULTRASONIC_TOPIC, 1);

    ultrasonicInit();

    sensors::Ultrasonic ult_data_cm;
    ros::Duration elapsed;
    bool success;

    while (ros::ok()) {
        success = readUltrasonic(ULTRASONIC_F_TRIG, ULTRASONIC_F_ECHO, elapsed_r);

        if (success) {
            ult_data_cm.data = (elapsed.toSec() * 34300) / 2;
    	    ROS_INFO("LEFT ELAPSED TIME: %f", elapsed.toSec());
        } else {
            ult_data_cm.data = sensors::Ultrasonic::INVALID_SENSOR_DATA;
            ROS_WARN("Timed out while reading ultrasonic sensor!");
        }

        ult_data_pub.publish(ult_data_cm);
    }
}
