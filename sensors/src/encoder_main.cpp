#include <ros/ros.h>
#include <vector>

#include "sensors/Encoder.h"
#include "sensors/GPIOClass.h"
#include "sensors/MotorEncoder.h"
#include "sensors/MotorEncoderISR.h"
#include "topics/topics.h"

#define MOTOR_R_ENCODER_A_PIN "16"
#define MOTOR_R_ENCODER_B_PIN "17"
#define MOTOR_L_ENCODER_A_PIN "24"
#define MOTOR_L_ENCODER_B_PIN "25"

int main(int argc, char **argv) {
    ros::init(argc, argv, "encoder");

    ros::NodeHandle nh;
    ros::Publisher encoder_data_pub = nh.advertise<sensors::Encoder>(topics::ENCODER_TOPIC, 1);

    MotorEncoder encoder_r = MotorEncoder(MOTOR_R_ENCODER_A_PIN, MOTOR_R_ENCODER_B_PIN, &MotorEncoderISR::ISR1, &MotorEncoderISR::rising_edges_1);
    MotorEncoder encoder_l = MotorEncoder(MOTOR_L_ENCODER_A_PIN, MOTOR_L_ENCODER_B_PIN, &MotorEncoderISR::ISR2, &MotorEncoderISR::rising_edges_2);

    ros::Rate rate(100);
    sensors::Encoder encoder_msg;
    std::vector<float> encoder_data;
    encoder_data.resize(2);
    while (ros::ok()) {
        encoder_data[1] = encoder_r.positionInDegrees();
        encoder_data[2] = encoder_l.positionInDegrees();
        encoder_msg.data = encoder_data;
        encoder_data_pub.publish(encoder_msg);
        rate.sleep();
    }

    return 0;
}