#include <ros/ros.h>
#include <vector>

#include "constants/gpio_pins.h"
#include "constants/topics.h"
#include "external/wiringPi/wiringPi.h"
#include "sensors/Encoder.h"
#include "sensors/MotorEncoder.h"
#include "sensors/MotorEncoderISR.h"

const int COUNTS_PER_REV = 480;

bool encoderSetup() {
    if (wiringPiSetup() < 0) {
        ROS_ERROR("Unable to set up wiringPi");
        return false;
    }
    pinMode(MOTOR_R_ENCODER_A_PIN, INPUT);
    pinMode(MOTOR_R_ENCODER_B_PIN, INPUT);
    // pinMode(MOTOR_L_ENCODER_A_PIN, INPUT);
    // pinMode(MOTOR_L_ENCODER_B_PIN, INPUT);

    // ISR setup
    // if (wiringPiISR(MOTOR_R_ENCODER_A_PIN, INT_EDGE_RISING, &MotorEncoderISR::ISR1) < 0) {
    //     ROS_ERROR("Unable to set up wiringPiISR for motor encoder A");
    //     return false;
    // }
    // if (wiringPiISR(MOTOR_L_ENCODER_A_PIN, INT_EDGE_RISING, &MotorEncoderISR::ISR2) < 0) {
    //     ROS_ERROR("Unable to set up wiringPiISR for motor encoder B");
    //     return false;
    // }
    return true;
}  

int main(int argc, char **argv) {
    ros::init(argc, argv, "encoder");
    ros::NodeHandle nh;
    ros::Publisher encoder_data_pub = nh.advertise<sensors::Encoder>
        (topics::ENCODER_TOPIC, 1);
    
    if (!encoderSetup()) {
        return -1;
    }

    sensors::Encoder encoder_msg;
    std::vector<float> encoder_data;
    encoder_data.push_back(0);
    encoder_data.push_back(0);

    ros::Time time = ros::Time::now();
    int encoder_count = 0;
    const int NUM_REVOLUTIONS_TO_COUNT = 10;

    bool a_last_state = digitalRead(MOTOR_R_ENCODER_A_PIN);
    bool a_state = a_last_state = false;
    bool n = false;
    int counter = 0;
    ros::Time prev_time = ros::Time::now();
    while (ros::ok()) {
        n = digitalRead(MOTOR_R_ENCODER_A_PIN);        
	// Rising edge detected
        if  ((a_last_state == 0) && n) {

		if (digitalRead(MOTOR_R_ENCODER_B_PIN)) {
        	    ++counter;
        	} else {
         	   	--counter;
        	}
        }
	if (counter == COUNTS_PER_REV*NUM_REVOLUTIONS_TO_COUNT || counter == -COUNTS_PER_REV*NUM_REVOLUTIONS_TO_COUNT) {
            ROS_INFO("RPM: %f", NUM_REVOLUTIONS_TO_COUNT / (ros::Time::now() - prev_time).toSec() * 60.0);
            counter = 0;
            prev_time = ros::Time::now();
        }
    	a_last_state = n;
    }

    return 0;
}

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "encoder");

//     ros::NodeHandle nh;
//     ros::Publisher encoder_data_pub = nh.advertise<sensors::Encoder>
//         (topics::ENCODER_TOPIC, 1);

//     MotorEncoder encoder_r = MotorEncoder(MOTOR_R_ENCODER_A_PIN,
//         MOTOR_R_ENCODER_B_PIN,
//         &MotorEncoderISR::ISR1,
//         &MotorEncoderISR::rising_edges_1);
//     MotorEncoder encoder_l = MotorEncoder(MOTOR_L_ENCODER_A_PIN,
//         MOTOR_L_ENCODER_B_PIN,
//         &MotorEncoderISR::ISR2,
//         &MotorEncoderISR::rising_edges_2);

//     ros::Rate rate(100);
//     sensors::Encoder encoder_msg;
//     std::vector<float> encoder_data;
//     encoder_data.resize(2);
//     while (ros::ok()) {
//         if (!encoder_r.updateCount()) {
//             ROS_WARN("Unable to update right encoder count, publishing previous data");
//         }
//         if (!encoder_l.updateCount()) {
//             ROS_WARN("Unable to update left encoder count, publishing previous data");
//         }

//         encoder_data[0] = encoder_r.positionInDegrees();
//         encoder_data[1] = encoder_l.positionInDegrees();
//         encoder_msg.data = encoder_data;
//         encoder_data_pub.publish(encoder_msg);
//         if (encoder_data[0] >= 270 || encoder_data[1] >= 270) return 0;
//         rate.sleep();
//     }

//     return 0;
// }
