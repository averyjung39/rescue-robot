#include <ros/ros.h>

#include "localization/ParticleFilter.h"
#include "localization/Pose.h"
#include "sensors/Encoder.h"
#include "sensors/IMU.h"
#include "sensors/Ultrasonic.h"
#include "constants/topics.h"

sensors::Ultrasonic ultrasonic_msg;
sensors::Encoder encoder_msg;
sensors::IMU imu_msg;

void ultrasonicCallback(const sensors::Ultrasonic::ConstPtr &msg) {
    ultrasonic_msg = *msg;
}

void encoderCallback(const sensors::Encoder::ConstPtr &msg) {
    encoder_msg = *msg;
}

void imuCallback(const sensors::IMU::ConstPtr &msg) {
    imu_msg = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization");

    ros::NodeHandle nh;
    ros::Subscriber ultrasonic_sub = nh.subscribe(topics::ULTRASONIC_TOPIC, 1, ultrasonicCallback);
    ros::Subscriber encoder_sub = nh.subscribe(topics::ENCODER_TOPIC, 1, encoderCallback);
    ros::Subscriber imu_sub = nh.subscribe(topics::IMU_TOPIC, 1, imuCallback);
    ros::Publisher pose_pub = nh.advertise<localization::Pose>(topics::ROBOT_POSE_TOPIC, 1);

    ParticleFilter particle_filter = ParticleFilter(500, 0.1, 0.1, 0.5); // TODO validate params for this
    localization::Pose pose;
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        pose = particle_filter.getPoseEstimate(encoder_msg, imu_msg, ultrasonic_msg);
        rate.sleep();
    }

    return 0;
}