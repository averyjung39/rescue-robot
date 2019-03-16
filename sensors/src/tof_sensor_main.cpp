#include <ros/ros.h>
#include <ros/duration.h>
#include <vector>

#include "sensors/Distance.h"
#include "constants/topics.h"
#include "constants/gpio_pins.h"
#include "external/wiringPi/wiringPi.h"
#include "external/tof/VL53L0X.h"

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
VL53L0X tof3 = VL53L0X();
VL53L0X tof4 = VL53L0X();
VL53L0X tof5 = VL53L0X();

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

bool readdress() {
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }

    pinMode(TOF_XSHUT_1, OUTPUT);
    pinMode(TOF_XSHUT_2, OUTPUT);
    pinMode(TOF_XSHUT_3, OUTPUT);
    pinMode(TOF_XSHUT_4, OUTPUT);
    pinMode(TOF_XSHUT_5, OUTPUT);

    ROS_INFO("Shutdown pins...");

    digitalWrite(TOF_XSHUT_1, LOW);
    digitalWrite(TOF_XSHUT_2, LOW);
    digitalWrite(TOF_XSHUT_3, LOW);
    digitalWrite(TOF_XSHUT_4, LOW);
    digitalWrite(TOF_XSHUT_5, LOW);

    ROS_INFO("All sensors in reset mode...(pins are low)");

    ros::Duration(0.01).sleep();

    if (!setId(tof1, TOF_ADDR_1, TOF_XSHUT_1)) { return false; }
    if (!setId(tof2, TOF_ADDR_2, TOF_XSHUT_2)) { return false; }
    if (!setId(tof3, TOF_ADDR_3, TOF_XSHUT_3)) { return false; }
    if (!setId(tof4, TOF_ADDR_4, TOF_XSHUT_4)) { return false; }
    if (!setId(tof5, TOF_ADDR_5, TOF_XSHUT_5)) { return false; }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tof_sensor");

    ros::NodeHandle nh;

    ros::Publisher low_dist_pub = nh.advertise<sensors::Distance>(topics::LOW_DIST_TOPIC, 1);
    ros::Publisher high_dist_pub = nh.advertise<sensors::Distance>(topics::HIGH_DIST_TOPIC, 1);
    int tof_distance1, tof_distance2, tof_distance3, tof_distance4, tof_distance5;
    int model, revision;

    // Readdress ToF sensors
    readdress();

    ROS_INFO("VL53L0X device successfully opened.\n");

    sensors::Distance low_dist_data_cm, high_dist_data_cm;
    low_dist_data_cm.data.resize(3);
    high_dist_data_cm.data.resize(3);
    ros::Rate rate(10);

    while (ros::ok()) {
        // Read data from the sensors
        tof_distance1 = tof1.tofReadDistance();
	    tof_distance2 = tof2.tofReadDistance();
        tof_distance3 = tof3.tofReadDistance();
        tof_distance4 = tof4.tofReadDistance();
        tof_distance5 = tof5.tofReadDistance();


        // Check if they are in valid range and populate the ToF data msg
        low_dist_data_cm.data[0] = (tof_distance1 < 4096) ? tof_distance1 / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;
        low_dist_data_cm.data[1] = (tof_distance2 < 4096) ? tof_distance2 / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;
        low_dist_data_cm.data[2] = (tof_distance3 < 4096) ? tof_distance3 / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;
        high_dist_data_cm.data[0] = (tof_distance4 < 4096) ? tof_distance4 / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;
        high_dist_data_cm.data[1] = (tof_distance5 < 4096) ? tof_distance5 / 10.0 : sensors::Distance::INVALID_SENSOR_DATA;

        low_dist_pub.publish(low_dist_data_cm);
        high_dist_pub.publish(high_dist_data_cm);
        rate.sleep();
    }

    return 0;
}
