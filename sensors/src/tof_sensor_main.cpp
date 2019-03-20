#include <ros/ros.h>
#include <ros/duration.h>
#include <vector>

#include "sensors/Distance.h"
#include "constants/topics.h"
#include "constants/gpio_pins.h"
#include "external/wiringPi/wiringPi.h"
#include "external/tof/VL53L0X.h"

#define MAX_TOF 4096

/*
1. Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
2. Keep sensor #1 awake by keeping XSHUT pin high
3. Put all other sensors into shutdown by pulling XSHUT pins low
4. Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Goingwith 0x30 to 0x3F is probably OK.
5. Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
6. Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
Don't forget to remove the protective plastic cover from the sensor before using!
7. Repeat for each sensor, turning each one on, setting a unique address.Note you must do this every time you turn on the power, the addresses are not permanent*/

enum TOF {
    BOTTOM_LEFT = 1,
    BOTTOM_RIGHT = 2,
    TOP_FRONT = 3,
    TOP_BACK = 4,
    TOP_LEFT = 5,
    TOP_RIGHT = 6
};

VL53L0X b_left = VL53L0X();
VL53L0X b_right = VL53L0X();
VL53L0X t_front = VL53L0X();
VL53L0X t_back = VL53L0X();
VL53L0X t_left = VL53L0X();
VL53L0X t_right = VL53L0X();

#define I2C_SLAVE_DEVICE_ADDRESS 0x8A

bool setId(VL53L0X &tof, uint8_t address, int xshut_pin) {
    digitalWrite(xshut_pin, HIGH);
    ros::Duration(0.01).sleep();
    tof.tofInit(1);
    ros::Duration(0.01).sleep();
    if (!tof.setAddress(address)) {
        ROS_ERROR("Failed to set address %x. Connected to GPIO %d", address, xshut_pin);
        return false;
    }
    ros::Duration(0.01).sleep();
    ROS_INFO("Successfully readdressed ToF to %x", tof.readReg(I2C_SLAVE_DEVICE_ADDRESS));
    return true;
}

bool readdress(bool all = true, int sensor = 0) {
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }

    pinMode(TOF_BOTTOM_LEFT, OUTPUT);
    pinMode(TOF_BOTTOM_RIGHT, OUTPUT);
    pinMode(TOF_TOP_FRONT, OUTPUT);
    pinMode(TOF_TOP_BACK, OUTPUT);
    pinMode(TOF_TOP_LEFT, OUTPUT);
    pinMode(TOF_TOP_RIGHT, OUTPUT);

    ROS_INFO("Shutdown pins...");

    digitalWrite(TOF_BOTTOM_LEFT, LOW);
    digitalWrite(TOF_BOTTOM_RIGHT, LOW);
    digitalWrite(TOF_TOP_FRONT, LOW);
    digitalWrite(TOF_TOP_BACK, LOW);
    digitalWrite(TOF_TOP_LEFT, LOW);
    digitalWrite(TOF_TOP_RIGHT, LOW);

    ROS_INFO("All sensors in reset mode...(pins are low)");

    ros::Duration(0.01).sleep();

    if (all || sensor == BOTTOM_LEFT) {
        if (!setId(b_left, BOTTOM_LEFT_ADDR, TOF_BOTTOM_LEFT)) { return false; }
    }
    if (all || sensor == BOTTOM_RIGHT) {
        if (!setId(b_right, BOTTOM_RIGHT_ADDR, TOF_BOTTOM_RIGHT)) { return false; }
    }
    if (all || sensor == TOP_FRONT) {
        if (!setId(t_front, TOP_FRONT_ADDR, TOF_TOP_FRONT)) { return false; }
    }
    if (all || sensor == TOP_BACK) {
        if (!setId(t_back, TOP_BACK_ADDR, TOF_TOP_BACK)) { return false; }
    }
    if (all || sensor == TOP_LEFT) {
        if (!setId(t_left, TOP_LEFT_ADDR, TOF_TOP_LEFT)) { return false; }
    }
    if (all || sensor == TOP_RIGHT) {
        if (!setId(t_right, TOP_RIGHT_ADDR, TOF_TOP_RIGHT)) { return false; }
    }

    digitalWrite(TOF_BOTTOM_LEFT, HIGH);
    digitalWrite(TOF_BOTTOM_RIGHT, HIGH);
    digitalWrite(TOF_TOP_FRONT, HIGH);
    digitalWrite(TOF_TOP_BACK, HIGH);
    digitalWrite(TOF_TOP_LEFT, HIGH);
    digitalWrite(TOF_TOP_RIGHT, HIGH);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tof_sensor");

    ros::NodeHandle nh;

    ros::Publisher low_dist_pub = nh.advertise<sensors::Distance>(topics::LOW_DIST_TOPIC, 1);
    ros::Publisher high_dist_pub = nh.advertise<sensors::Distance>(topics::HIGH_DIST_TOPIC, 1);
    int b_left_dist, b_right_dist, t_front_dist, t_back_dist, t_left_dist, t_right_dist;
    int model, revision;

    // Readdress ToF sensors
    readdress();

    ROS_INFO("VL53L0X device successfully opened.\n");

    sensors::Distance low_dist_data_cm, high_dist_data_cm;
    low_dist_data_cm.data.resize(2);
    high_dist_data_cm.data.resize(4);
    ros::Rate rate(10);

    while (ros::ok()) {
        // Read data from the sensors
        b_left_dist = b_left.tofReadDistance();
        b_right_dist = b_right.tofReadDistance();
        t_front_dist = t_front.tofReadDistance();
        t_back_dist = t_back.tofReadDistance();
        t_left_dist = t_left.tofReadDistance();
        t_right_dist = t_right.tofReadDistance();

        if (b_left_dist == -1) {
            b_left = VL53L0X();
            readdress(false, BOTTOM_LEFT);
        }
        if (b_right_dist == -1) {
            b_right = VL53L0X();
            readdress(false, BOTTOM_RIGHT);
        }
        if (t_front_dist == -1) {
            t_front = VL53L0X();
            readdress(false, TOP_FRONT);
        }
        if (t_back_dist == -1) {
            t_back = VL53L0X();
            readdress(false, TOP_BACK);
        }
        if (t_left_dist == -1) {
            t_left = VL53L0X();
            readdress(false, TOP_LEFT);
        }
        if (t_right_dist == -1) {
            t_right = VL53L0X();
            readdress(false, TOP_RIGHT);
        }


        // Check if they are in valid range and populate the ToF data msg
        low_dist_data_cm.data[0] = (b_left_dist < MAX_TOF && b_left_dist != -1) ? b_left_dist / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;
        low_dist_data_cm.data[1] = (b_right_dist < MAX_TOF && b_right_dist != -1) ? b_right_dist / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;
        high_dist_data_cm.data[0] = (t_front_dist < MAX_TOF && t_front_dist != -1) ? t_front_dist / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;
        high_dist_data_cm.data[1] = (t_back_dist < MAX_TOF && t_back_dist != -1) ? t_back_dist / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;
        high_dist_data_cm.data[2] = (t_left_dist < MAX_TOF && t_left_dist != -1) ? t_left_dist / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;
        high_dist_data_cm.data[3] = (t_right_dist < MAX_TOF && t_right_dist != -1) ? t_right_dist / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;

        low_dist_pub.publish(low_dist_data_cm);
        // high_dist_pub.publish(high_dist_data_cm);
        rate.sleep();
    }

    return 0;
}
