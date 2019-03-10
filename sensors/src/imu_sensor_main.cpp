#include <ros/ros.h>
#include <vector>

#include "constants/topics.h"
#include "external/wiringPi/wiringPiI2C.h"
#include "external/IMU/IMUSensor.h"
#include "sensors/IMU.h"

#define G_GAIN 0.070       // [deg/s/LSB]
#define CF_GYR_ACC 0.98    // complementary filter constant for gyro and accelerometer
#define CF_GYR_MAG 0.95    // complementary filter constant for gyro and magnetometer

/**
 * @brief Read raw data from IMU
 * @param acc_data_raw, gyr_data_raw, mag_data_raw, return parameters for raw
 *        accelerometer/gyroscope/magnetometer
 */
void readIMURaw(int (&acc_data_raw)[3], int (&gyr_data_raw)[3], int (&mag_data_raw)[3]) {
    IMU::readACC(acc_data_raw);
    IMU::readGYR(gyr_data_raw);
    IMU::readMAG(mag_data_raw);
}

/**
 * @brief Read IMU (ideally converted from raw data)
 * @param acc_data Return parameter for raw accelerometer data
 * @param gyr_data Return parameter for gyroscope data in deg/s
 * @param mag_data Return parameter for raw magnetometer data
 */
void readIMU(float (&acc_data)[3], float (&gyr_data)[3], float (&mag_data)[3]) {
    // Read raw data
    int acc_data_raw[3] = {0};
    int gyr_data_raw[3] = {0};
    int mag_data_raw[3] = {0}; // Currently not using
    readIMURaw(acc_data_raw, gyr_data_raw, mag_data_raw);

    // Convert values to readable units

    // TODO figure out how to convert to m/s^2
    // In theory if we are only looking for changes in z-acc, we could
    // also just work with raw values
    for (int i = 0; i < 3; ++i) {
        acc_data[i] = acc_data_raw[i];
    }

    // Gyro to deg / s
    // https://github.com/mwilliams03/Raspberry-Gyro-Acc/blob/master/main.c#L116

    for (int i = 0; i < 3; ++i) {
        gyr_data[i] = gyr_data_raw[i] * G_GAIN;
    }

    // Mag raw values
    for (int i = 0; i < 3; ++i) {
        mag_data[i] = mag_data_raw[i];
    }
}

/**
 * @brief Calculate roll, pitch, yaw orientation of IMU
 * @param acc_data, gyr_data, mag_data Data from IMU
 * @param gyr_angles Gyroscope angles in degrees
 * @param loop_frequency Frequency of main loop in Hz
 * @param orientation Return parameter for roll, pitch, yaw orientation in degrees
 */
void getOrientation(const float (&acc_data)[3],
    const float (&gyr_data)[3],
    const float (&mag_data)[3],
    float (&gyr_angles)[3],
    const float &loop_frequency,
    std::vector<float> &orientation) {

    // Gyroscope angle update
    // https://github.com/mwilliams03/Raspberry-Gyro-Acc/blob/master/main.c#L123
    for (int i = 0; i < 3; ++i) {
        gyr_angles[i] += gyr_data[i] / loop_frequency;
    }

    // Accelerometer angles
    // https://github.com/mwilliams03/Raspberry-Gyro-Acc/blob/master/main.c#L131
    float acc_roll = atan2(acc_data[1], acc_data[2]) * 180 / M_PI;
    float acc_pitch = atan2(acc_data[2], acc_data[0]) * 180 / M_PI;
    
    // Magnetometer angle
    float mag_yaw = atan2(mag_data[1], mag_data[0]) * 180 / M_PI;

    // Combine angle readings
    orientation[0] = CF_GYR_ACC*(orientation[0] + gyr_angles[0]) + (1 - CF_GYR_ACC)*acc_roll;  // Roll
    orientation[1] = CF_GYR_ACC*(orientation[1] + gyr_angles[1]) + (1 - CF_GYR_ACC)*acc_pitch; // Pitch
    orientation[2] = CF_GYR_MAG*(orientation[2] + gyr_angles[2]) + (1 - CF_GYR_MAG)*mag_yaw;   // Yaw
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_sensor");
    ros::NodeHandle nh;

    IMU::enableIMU();
    float acc_data[3] = {0};
    float gyr_data[3] = {0};
    float mag_data[3] = {0};

    ros::Publisher imu_pub = nh.advertise<sensors::IMU>(topics::IMU_TOPIC, 1);

    sensors::IMU imu_msg;
    float loop_frequency = 20;
    ros::Rate rate(loop_frequency);
    float gyr_angles[3] = {0.0};
    std::vector<float> orientation;
    for (int i = 0; i < 3; ++i) {
        orientation.push_back(0);
    }
    while (ros::ok()) {
        readIMU(acc_data, gyr_data, mag_data);
        getOrientation(acc_data, gyr_data, mag_data, gyr_angles, loop_frequency, orientation);

        imu_msg.x_acc = acc_data[0];
        imu_msg.y_acc = acc_data[1];
        imu_msg.z_acc = acc_data[2];
        imu_msg.roll = orientation[0];
        imu_msg.pitch = orientation[1];
        imu_msg.yaw = orientation[2];
        imu_pub.publish(imu_msg);
        rate.sleep();
    }

    return 0;
}