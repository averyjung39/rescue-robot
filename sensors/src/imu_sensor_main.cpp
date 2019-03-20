#include <ros/ros.h>
#include <vector>

#include "constants/topics.h"
#include "external/wiringPi/wiringPiI2C.h"
#include "external/IMU/IMUSensor.h"
#include "sensors/IMU.h"

#define G_GAIN 0.07       // [deg/s/LSB]
#define GYR_X_ZERO_OFFSET 2.461824  // 0.307728  // Experimentally determined, deg/s
#define GYR_Y_ZERO_OFFSET 1.460976  // 0.182622  // TODO: calculate these at runtime
#define GYR_Z_ZERO_OFFSET 0.960920  // 0.120115
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
    gyr_data[0] = gyr_data[1] = gyr_data[2] = 0;
    int NUM_GYR_ITERATIONS = 10;
    for (int j = 0; j < NUM_GYR_ITERATIONS; ++j) {
        for (int i = 0; i < 3; ++i) {
            IMU::readGYR(gyr_data_raw);
            gyr_data[i] += gyr_data_raw[i] * G_GAIN;
        }
    }
    // Subtract zero offset
    for (int i = 0; i < 3; ++i) {
        gyr_data[i] /= NUM_GYR_ITERATIONS;
    }
    gyr_data[0] -= GYR_X_ZERO_OFFSET;
    gyr_data[1] -= GYR_Y_ZERO_OFFSET;
    gyr_data[2] -= GYR_Z_ZERO_OFFSET;
    // ROS_INFO("gyr_data [deg/s]: %f, %f, %f", gyr_data[0], gyr_data[1], gyr_data[2]);

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
    // TODO measure time elapsed to make this more accurate
    for (int i = 0; i < 3; ++i) {
        gyr_angles[i] += gyr_data[i] / loop_frequency;
    }
    ROS_INFO("gyr_angles: %f, %f, %f", gyr_angles[0], gyr_angles[1], gyr_angles[2]);

    // Accelerometer angles
    // https://github.com/mwilliams03/Raspberry-Gyro-Acc/blob/master/main.c#L131
    float acc_roll = atan2(acc_data[1], sqrt(acc_data[2]*acc_data[2] + acc_data[0]*acc_data[0])) * 180 / M_PI;
    float acc_pitch = atan2(acc_data[0], sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2])) * 180 / M_PI;

    // Magnetometer angle (with tilt compensation)
    // // float mag_yaw = atan2(mag_data[1], mag_data[0]) * 180 / M_PI;
    // float mag_norm = sqrt(mag_data[0]*mag_data[0] + mag_data[1]*mag_data[1] + mag_data[2]*mag_data[2]);
    // float mag_x = mag_data[0] / mag_norm;
    // float mag_y = mag_data[1] / mag_norm;
    // float mag_z = mag_data[2] / mag_norm;
    // float roll_rad = acc_roll * M_PI / 180;
    // float pitch_rad = acc_pitch * M_PI / 180;
    // mag_x = mag_x * cos(-pitch_rad) + mag_z * sin(-pitch_rad);
    // mag_y = mag_x * sin(roll_rad) * sin(-pitch_rad) + mag_y * cos(roll_rad) - mag_z * sin(roll_rad) * cos(-pitch_rad);
    // float mag_yaw = atan2(mag_y, mag_x) * 180 / M_PI;
    // ROS_INFO("roll: %f, pitch: %f, yaw: %f", acc_roll, acc_pitch, mag_yaw);

    // Combine angle readings
    orientation[0] = CF_GYR_ACC*(orientation[0] + gyr_angles[0]) + (1 - CF_GYR_ACC)*acc_roll;  // Roll
    orientation[1] = CF_GYR_ACC*(orientation[1] + gyr_angles[1]) + (1 - CF_GYR_ACC)*acc_pitch; // Pitch
    orientation[2] = gyr_angles[2];   // Yaw
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
