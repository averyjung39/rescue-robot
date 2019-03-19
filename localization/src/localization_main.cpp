#include <ros/ros.h>
#include <vector>

#include "constants/topics.h"
#include "messages/Arc.h"
#include "localization/Pose.h"
#include "sensors/Distance.h"
#include "sensors/IMU.h"

int command_type = messages::Arc::STOP;
float imu_yaw = 0;
std::vector<float> high_dist_data;


void controlCommandCallback(const messages::Arc::ConstPtr &msg) {
    command_type = msg->command_type;
}

void highDistanceCallback(const sensors::Distance::ConstPtr &msg) {
    high_dist_data = msg->data;
}

void imuCallback(const sensors::IMU::ConstPtr &msg) {
    imu_yaw = msg->yaw;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_localization");
    ros::NodeHandle nh;

    ros::Subscriber control_command_sub = nh.subscribe(topics::ARC_TOPIC, 1, controlCommandCallback);
    ros::Subscriber high_distance_sub = nh.subscribe(topics::HIGH_DIST_TOPIC, 1, highDistanceCallback);
    ros::Subscriber imu_sub = nh.subscribe(topics::IMU_TOPIC, 1, imuCallback);
    ros::Publisher pose_pub = nh.advertise<localization::Pose>(topics::POSE_TOPIC, 1);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}