#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include "sensors/Ultrasonic.h"
#include "constants/gpio_pins.h"
#include "constants/topics.h"
#include "external/wiringPi/wiringPi.h"

void ultrasonicInit(){
    pinMode(ULTRASONIC_R_TRIG, OUTPUT);
    pinMode(ULTRASONIC_R_ECHO, INPUT);
    pinMode(ULTRASONIC_L_TRIG, OUTPUT);
    pinMode(ULTRASONIC_L_ECHO, INPUT);
    pinMode(ULTRASONIC_B_TRIG, OUTPUT);
    pinMode(ULTRASONIC_B_ECHO, INPUT);
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
    ult_data_cm.data.resize(3);

    ros::Duration elapsed_r;
    ros::Duration elapsed_l;
    ros::Duration elapsed_b;

    bool success_r;
    bool success_l;
    bool success_b;

    while (ros::ok()) {
        success_r = readUltrasonic(ULTRASONIC_R_TRIG, ULTRASONIC_R_ECHO, elapsed_r);
        success_l = readUltrasonic(ULTRASONIC_L_TRIG, ULTRASONIC_L_ECHO, elapsed_l);
        success_b = readUltrasonic(ULTRASONIC_B_TRIG, ULTRASONIC_B_ECHO, elapsed_b);

        if (success_l) {
            ult_data_cm.data[0] = (elapsed_l.toSec() * 34300) / 2;
    	    ROS_INFO("LEFT ELAPSED TIME: %f", elapsed_l.toSec());
        } else {
            ult_data_cm.data[0] = sensors::Ultrasonic::INVALID_SENSOR_DATA;
            ROS_WARN("Timed out while reading left ultrasonic sensor!");
        }

        if (success_b) {
            ult_data_cm.data[1] = (elapsed_b.toSec() * 34300) / 2;
            ROS_INFO("BACK ELAPSED TIME: %f", elapsed_b.toSec());
        } else {
            ult_data_cm.data[1] = sensors::Ultrasonic::INVALID_SENSOR_DATA;
            ROS_WARN("Timed out while reading back ultrasonic sensor!");
        }

        if (success_r) {
            ult_data_cm.data[2] = (elapsed_r.toSec() * 34300) / 2;
            ROS_INFO("RIGHT ELAPSED TIME: %f", elapsed_r.toSec());
        } else {
            ult_data_cm.data[2] = sensors::Ultrasonic::INVALID_SENSOR_DATA;
            ROS_WARN("Timed out while reading right ultrasonic sensor!");
        }

        ult_data_pub.publish(ult_data_cm);
    }
}
