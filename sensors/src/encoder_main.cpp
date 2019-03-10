#include <ros/ros.h>
#include <vector>

#include "constants/gpio_pins.h"
#include "constants/topics.h"
#include "sensors/Encoder.h"
#include "sensors/MotorEncoder.h"
#include "sensors/MotorEncoderISR.h"



int main(int argc, char **argv) {
    ros::init(argc, argv, "encoder");

    ros::NodeHandle nh;
    ros::Publisher encoder_data_pub = nh.advertise<sensors::Encoder>
        (topics::ENCODER_TOPIC, 1);

    MotorEncoder encoder_r = MotorEncoder(MOTOR_R_ENCODER_A_PIN,
        MOTOR_R_ENCODER_B_PIN,
        &MotorEncoderISR::ISR1,
        &MotorEncoderISR::rising_edges_1);
    MotorEncoder encoder_l = MotorEncoder(MOTOR_L_ENCODER_A_PIN,
        MOTOR_L_ENCODER_B_PIN,
        &MotorEncoderISR::ISR2,
        &MotorEncoderISR::rising_edges_2);

    ros::Rate rate(100);
    sensors::Encoder encoder_msg;
    std::vector<float> encoder_data;
    encoder_data.resize(2);
    while (ros::ok()) {
        if (!encoder_r.updateCount()) {
            ROS_WARN("Unable to update right encoder count, publishing previous data");
        }
        if (!encoder_l.updateCount()) {
            ROS_WARN("Unable to update left encoder count, publishing previous data");
        }

        encoder_data[1] = encoder_r.positionInDegrees();
        encoder_data[2] = encoder_l.positionInDegrees();
        encoder_msg.data = encoder_data;
        encoder_data_pub.publish(encoder_msg);

        rate.sleep();
    }

    return 0;
}