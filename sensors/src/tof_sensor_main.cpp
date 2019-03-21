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

VL53L0X tof1 = VL53L0X();
VL53L0X tof2 = VL53L0X();
VL53L0X tof3 = VL53L0X();
//VL53L0X tof4 = VL53L0X();
//VL53L0X tof5 = VL53L0X();

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
//    pinMode(TOF_XSHUT_4, OUTPUT);
//    pinMode(TOF_XSHUT_5, OUTPUT);

    ROS_INFO("Shutdown pins...");

    digitalWrite(TOF_XSHUT_1, LOW);
    digitalWrite(TOF_XSHUT_2, LOW);
    digitalWrite(TOF_XSHUT_3, LOW);
//    digitalWrite(TOF_XSHUT_4, LOW);
//    digitalWrite(TOF_XSHUT_5, LOW);

    ROS_INFO("All sensors in reset mode...(pins are low)");

    ros::Duration(0.01).sleep();

    if (!setId(tof1, TOF_ADDR_1, TOF_XSHUT_1)) { return false; }
    if (!setId(tof2, TOF_ADDR_2, TOF_XSHUT_2)) { return false; }
   // if (!setId(tof3, TOF_ADDR_3, TOF_XSHUT_3)) { return false; }
//    if (!setId(tof4, TOF_ADDR_4, TOF_XSHUT_4)) { return false; }
//    if (!setId(tof5, TOF_ADDR_5, TOF_XSHUT_5)) { return false; }

    return true;
}

float denoiseTof(std::vector<int> data, TOF sensor) {
    float sum_data = 0.0;
    int num_useful_data = data.size();
    for(int i = 0; i < data.size(); i++) {
        // INVALID_SENSOR_DATA
        if (data[i] >= MAX_TOF) {
            num_useful_data--;
            continue;
        } else if (data[i] == -1) {
            // failed to read from the bus
            // assume the sensor got reset to default address and set address again
            switch(sensor) {
                case BOTTOM_LEFT:
                    b_left = VL53L0X();
                    readdress(false, BOTTOM_LEFT);
                    break;
                case BOTTOM_RIGHT:
                    b_right = VL53L0X();
                    readdress(false, BOTTOM_RIGHT);
                    break;
                case TOP_FRONT:
                    t_front = VL53L0X();
                    readdress(false, TOP_FRONT);
                    break;
                case TOP_BACK:
                    t_back = VL53L0X();
                    readdress(false, TOP_BACK);
                    break;
                case TOP_LEFT:
                    t_left = VL53L0X();
                    readdress(false, TOP_LEFT);
                    break;
                case TOP_RIGHT:
                    t_right = VL53L0X();
                    readdress(false, TOP_RIGHT);
                    break;
                default:
                    ROS_ERROR("Unknown sensor");
                    break;
                return sensors::Distance::INVALID_SENSOR_DATA;
            }
        }
        sum_data += data[i];
    }
    if (num_useful_data <= 2) {
        return sensors::Distance::INVALID_SENSOR_DATA;
    }
    // return the average value in cm
    return (sum_data / num_useful_data) / 10.0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tof_sensor");

    ros::NodeHandle nh;

    ros::Publisher low_dist_pub = nh.advertise<sensors::Distance>(topics::LOW_DIST_TOPIC, 1);
    ros::Publisher high_dist_pub = nh.advertise<sensors::Distance>(topics::HIGH_DIST_TOPIC, 1);
    std::vector<int> b_left_dist, b_right_dist, t_front_dist, t_back_dist, t_left_dist, t_right_dist;
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
        for (int i = 0; i < 5; i++) {
            b_left_dist.push_back(b_left.tofReadDistance());
            b_right_dist.push_back(b_right.tofReadDistance());
            t_front_dist.push_back(t_front.tofReadDistance());
            t_back_dist.push_back(t_front.tofReadDistance());
            t_left_dist.push_back(t_left.tofReadDistance());
            t_right_dist.push_back(t_right.tofReadDistance());
        }

        // Denoise sensor value
        low_dist_data_cm.data[0] = denoiseTof(b_left_dist, BOTTOM_LEFT);
        low_dist_data_cm.data[1] = denoiseTof(b_right_dist, BOTTOM_RIGHT);
        high_dist_data_cm.data[0] = denoiseTof(t_front_dist, TOP_FRONT);
        high_dist_data_cm.data[1] = denoiseTof(t_back_dist, TOP_BACK);
        high_dist_data_cm.data[2] = denoiseTof(t_left_dist, TOP_LEFT);
        high_dist_data_cm.data[3] = denoiseTof(t_right_dist, TOP_RIGHT);

        low_dist_pub.publish(low_dist_data_cm);
        high_dist_pub.publish(high_dist_data_cm);
        rate.sleep();
    }

    return 0;
}
