#include <ros/ros.h>
#include <ros/duration.h>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "sensors/TimeOfFlight.h"
#include "sensors/tof.h"
#include "constants/topics.h"
#include "constants/gpio_pins.h"
#include "external/wiringPi/wiringPi.h"

/*
1. Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
2. Keep sensor #1 awake by keeping XSHUT pin high
3. Put all other sensors into shutdown by pulling XSHUT pins low
4. Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Goingwith 0x30 to 0x3F is probably OK.
5. Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
6. Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
Don't forget to remove the protective plastic cover from the sensor before using!
7. Repeat for each sensor, turning each one on, setting a unique address.Note you must do this every time you turn on the power, the addresses are not permanent*/

VL53L0X tof1 = VL53L0X();
VL53L0X tof2 = VL53L0X();

#define TOF_ADDR_1 0x30
#define TOF_ADDR_2 0x31
#define I2C_SLAVE_DEVICE_ADDRESS 0x8A

void setID() {
    digitalWrite(TOF_XSHUT_1, LOW);
    digitalWrite(TOF_XSHUT_2, LOW);
    ros::Duration(0.01).sleep();

    digitalWrite(TOF_XSHUT_1, HIGH);
    digitalWrite(TOF_XSHUT_2, HIGH);
    ros::Duration(0.01).sleep();

    digitalWrite(TOF_XSHUT_1, HIGH);
    digitalWrite(TOF_XSHUT_2, LOW);
    tof1.tofInit(1, TOF_ADDR_1, 1);
    ros::Duration(0.01).sleep();

    digitalWrite(TOF_XSHUT_2, HIGH);
    tof2.tofInit(1, TOF_ADDR_2, 1);
}

void setup() {
    pinMode(TOF_XSHUT_1, OUTPUT);
    pinMode(TOF_XSHUT_2, OUTPUT);

    ROS_INFO("Shutdown pins...");

    digitalWrite(TOF_XSHUT_1, LOW);
    digitalWrite(TOF_XSHUT_2, LOW);

    ROS_INFO("Both in reset mode...(pins are low)");

    setID();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tof_sensor");

    ros::NodeHandle nh;

    ros::Publisher tof_data_pub = nh.advertise<sensors::TimeOfFlight>(topics::TOF_TOPIC, 1);
    int tof_distance1, tof_distance2;
    int model, revision;

    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }

    // Readdress ToF sensors
    setup();
    ros::Duration(0.1).sleep();
    ROS_INFO("REG %x", tof1.readReg(I2C_SLAVE_DEVICE_ADDRESS));
    ROS_INFO("ADDR %x", tof1.getAddress());
    ROS_INFO("REG %x", tof2.readReg(I2C_SLAVE_DEVICE_ADDRESS));
    ROS_INFO("ADDR %x", tof2.getAddress());

    ROS_INFO("VL53L0X device successfully opened.\n");
    /*tof = tofGetModel(&model, &revision);
    ROS_INFO("Model ID - %d\n", model);
    ROS_INFO("Revision ID - %d\n", revision);*/

    sensors::TimeOfFlight tof_data_cm;
    tof_data_cm.data.resize(2);
    ros::Rate rate(10);

    while (ros::ok()) {
        /*
        tof_distance = tofReadDistance();
        if(tof_distance < 4096) { // valid range?
            tof_data_cm.data[0] = tof_distance / 10.0;
            tof_data_pub.publish(tof_data_cm);
        }*/
        rate.sleep();
    }

    return 0;
}
