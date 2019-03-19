#include <ros/ros.h>

#include "constants/topics.h"
#include "external/wiringPi/wiringPi.h"
#include "messages/Arc.h"
#include "sensors/IMU.h"
#include "sensors/Ultrasonic.h"

sensors::Ultrasonic ultrasonic_msg;
sensors::IMU imu_msg;

void ultrasonicCallback(const sensors::Ultrasonic::ConstPtr &msg) {
    ultrasonic_msg = *msg;
}

void imuCallback(const sensors::IMU::ConstPtr &msg) {
    imu_msg = *msg;
}

// Subscribe to ultrasonic message and make a 90 degree turn when detecting something 
// Use IMU to determine when 90 degrees has been turned
int main(int argc, char **argv) {
    if (wiringPiSetup() < 0 || wiringPiSetupGpio() < 0) {
        ROS_ERROR("Setting up wiringPi failed");
        return -1;
    }
    pinMode(45, INPUT);

    ros::init(argc, argv, "planning_test_turn_with_radius");
    ros::NodeHandle nh;

    ros::Subscriber ultrasonic_sub = nh.subscribe(topics::ULTRASONIC_TOPIC, 1, ultrasonicCallback);
    ros::Subscriber imu_sub = nh.subscribe(topics::IMU_TOPIC, 1, imuCallback);
    ros::Publisher arc_pub = nh.advertise<messages::Arc>(topics::ARC_TOPIC, 1);
    std::vector<float> ultrasonic_data;
    for (int i = 0; i < 3; ++i) {
        ultrasonic_data.push_back(sensors::Ultrasonic::INVALID_SENSOR_DATA);
    }
    ultrasonic_msg.data = ultrasonic_data;
    bool turn_now = false;
    messages::Arc arc_command;
    float speed;
    bool direction_is_right;
    nh.param<float>("speed", speed, 70);
    nh.param<bool>("direction_is_right", direction_is_right, true);
    arc_command.speed_r = speed;
    arc_command.speed_l = speed;
    arc_command.direction_is_right = direction_is_right;
    float yaw = 0;
    while (ros::ok()) {
        ros::spinOnce();
        if (turn_now) {
            turn_now = abs(imu_msg.yaw - yaw) <= 90;
            ROS_INFO("%f", imu_msg.yaw - yaw);
        } else {
            // Wait for gpio pin
            if (digitalRead(45)) {
                arc_command.radius = messages::Arc::TURN_ON_SPOT;
                turn_now = true;
                yaw = imu_msg.yaw;
            } else {
                arc_command.radius = messages::Arc::STOP;
            }
/*
            // Stay still and check ultrasonics
            for (int i = 0; i < 3; ++i) {
                if (ultrasonic_msg.data[i] < 10 && ultrasonic_msg.data[i] != sensors::Ultrasonic::INVALID_SENSOR_DATA) {
                    arc_command.radius = messages::Arc::TURN_ON_SPOT;
                    turn_now = true;
                    yaw = imu_msg.yaw;
                    break;
                } else {
                    arc_command.radius = messages::Arc::STOP;
                }
            }
*/
        }
        arc_pub.publish(arc_command);
    }
    return 0;
}
