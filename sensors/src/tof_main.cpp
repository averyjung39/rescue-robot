#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

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

bool setup() {
    if (wiringPiSetupGpio() == -1) {
        std::cout << "Setting up wiringPi failed." << std::endl;
        throw std::runtime_error("");
    }

    pinMode(TOF_XSHUT_1, OUTPUT);
    pinMode(TOF_XSHUT_2, OUTPUT);

    std::cout << "Shutdown pins..." << std::endl;

    digitalWrite(TOF_XSHUT_1, LOW);
    digitalWrite(TOF_XSHUT_2, LOW);

    std::cout << "Both in reset mode...(pins are low)" << std::endl;

    tof1.tofInit(1);
    tof2.tofInit(1);

    digitalWrite(TOF_XSHUT_1, LOW);
    digitalWrite(TOF_XSHUT_2, LOW);
    usleep(10);

    digitalWrite(TOF_XSHUT_1, HIGH);
    digitalWrite(TOF_XSHUT_2, HIGH);
    usleep(10);

    digitalWrite(TOF_XSHUT_1, HIGH);
    digitalWrite(TOF_XSHUT_2, LOW);
    usleep(10);
    if (!tof1.setAddress(TOF_ADDR_1)) {
        std::cout << "Failed to set address for ToF1. Connected to GPIO %d", TOF_XSHUT_1 << std::endl;
        return false;
    }
    usleep(10);

    digitalWrite(TOF_XSHUT_2, HIGH);
    usleep(10);
    if (!tof2.setAddress(TOF_ADDR_2)) {
        std::cout << "Failed to set address for ToF2. Connected to GPIO %d", TOF_XSHUT_2 << std::endl;
        return false;
    }

    return true;
}

int main(int argc, char **argv) {
    int tof_distance1, tof_distance2;
    int model, revision;

    // Readdress ToF sensors
    setup();
    usleep(10);
    ROS_INFO("REG %x", tof1.readReg(I2C_SLAVE_DEVICE_ADDRESS));
    ROS_INFO("ADDR %x", tof1.getAddress());
    ROS_INFO("REG %x", tof2.readReg(I2C_SLAVE_DEVICE_ADDRESS));
    ROS_INFO("ADDR %x", tof2.getAddress());

    ROS_INFO("VL53L0X device successfully opened.\n");

    sensors::TimeOfFlight tof_data_cm;

    return 0;
}
