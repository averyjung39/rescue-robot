#include <ros/ros.h>
#include <ros/duration.h>
#include <vector>
#include <std_msgs/Bool.h>

#include "sensors/Distance.h"
#include "constants/topics.h"
#include "constants/gpio_pins.h"
#include "external/wiringPi/wiringPi.h"
#include "external/tof/VL53L0X.h"

#define MAX_TOF 4096

bool fan_inactive = false;
bool readdress_done = false;
void fanCallback(const std_msgs::Bool::ConstPtr& msg) {
    fan_inactive = msg->data;
}

/*
1. Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
2. Keep sensor #1 awake by keeping XSHUT pin high
3. Put all other sensors into shutdown by pulling XSHUT pins low
4. Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Goingwith 0x30 to 0x3F is probably OK.
5. Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
6. Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
Don't forget to remove the protective plastic cover from the sensor before using!
7. Repeat for each sensor, turning each one on, setting a unique address.Note you must do this every time you turn on the power, the addresses are not permanent*/

VL53L0X b_left = VL53L0X();
VL53L0X b_right = VL53L0X();
VL53L0X t_front = VL53L0X();
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

bool readdress() {
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Setting up wiringPi failed.");
        throw std::runtime_error("");
    }

    pinMode(TOF_BOTTOM_LEFT, OUTPUT);
    pinMode(TOF_BOTTOM_RIGHT, OUTPUT);
    pinMode(TOF_TOP_FRONT, OUTPUT);
    pinMode(TOF_TOP_LEFT, OUTPUT);
    pinMode(TOF_TOP_RIGHT, OUTPUT);

    ROS_INFO("Shutdown pins...");

    digitalWrite(TOF_BOTTOM_LEFT, LOW);
    digitalWrite(TOF_BOTTOM_RIGHT, LOW);
    digitalWrite(TOF_TOP_FRONT, LOW);
    digitalWrite(TOF_TOP_LEFT, LOW);
    digitalWrite(TOF_TOP_RIGHT, LOW);

    ROS_INFO("All sensors in reset mode...(pins are low)");

    ros::Duration(0.01).sleep();

    bool success = true;
    if(!setId(b_left, BOTTOM_LEFT_ADDR, TOF_BOTTOM_LEFT)) { return false; }
    if (!setId(b_right, BOTTOM_RIGHT_ADDR, TOF_BOTTOM_RIGHT)) { return false; }
    if (!setId(t_front, TOP_FRONT_ADDR, TOF_TOP_FRONT)) { return false; }
    if (!setId(t_left, TOP_LEFT_ADDR, TOF_TOP_LEFT)) { return false; }
    if (!setId(t_right, TOP_RIGHT_ADDR, TOF_TOP_RIGHT)) { return false; }

    return success;
}

float denoiseTof(std::vector<int> data) {
    float sum_data = 0.0;
    int num_useful_data = data.size();
    for(int i = 0; i < data.size(); i++) {
        // INVALID_SENSOR_DATA
        if (data[i] >= MAX_TOF) {
            num_useful_data--;
            continue;
        } else if (data[i] == -1) {
            // failed to read from the bus
            return sensors::Distance::INVALID_SENSOR_DATA;
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
    bool print_data;
    nh.param<bool>("print_data", print_data, true);

    ros::Publisher low_dist_pub = nh.advertise<sensors::Distance>(topics::LOW_DIST_TOPIC, 1);
    ros::Publisher high_dist_pub = nh.advertise<sensors::Distance>(topics::HIGH_DIST_TOPIC, 1);
    std::vector<int> b_left_dist(3,0), b_right_dist(3,0), t_front_dist(3,0), t_back_dist(3,0), t_left_dist(3,0), t_right_dist(3,0);
    int model, revision;

    // Readdress ToF sensors
    readdress();

    ROS_INFO("VL53L0X device successfully opened.\n");

    sensors::Distance low_dist_data_cm, high_dist_data_cm;
    low_dist_data_cm.data.resize(2);
    high_dist_data_cm.data.resize(3);
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        if (fan_inactive && !readdress_done) {
            readdress();
            readdress_done = true;
        }
        // Read data from the sensors
        for (int i = 0; i < 3; i++) {
            b_left_dist[i] = b_left.tofReadDistance();
            b_right_dist[i] = b_right.tofReadDistance();
            t_front_dist[i] = t_front.tofReadDistance();
            t_left_dist[i] = t_left.tofReadDistance();
            t_right_dist[i] = t_right.tofReadDistance();
        }

        // Denoise sensor value
        low_dist_data_cm.data[0] = denoiseTof(b_left_dist);
        low_dist_data_cm.data[1] = denoiseTof(b_right_dist);
        high_dist_data_cm.data[0] = denoiseTof(t_front_dist);
        high_dist_data_cm.data[1] = denoiseTof(t_left_dist);
        high_dist_data_cm.data[2] = denoiseTof(t_right_dist);

        low_dist_pub.publish(low_dist_data_cm);
        high_dist_pub.publish(high_dist_data_cm);


       if (print_data){
         ROS_INFO("Data: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
           low_dist_data_cm.data[0],
           low_dist_data_cm.data[1],
           high_dist_data_cm.data[0],
           high_dist_data_cm.data[1],
           high_dist_data_cm.data[2]);
      }


       rate.sleep();
    }

    return 0;
}
