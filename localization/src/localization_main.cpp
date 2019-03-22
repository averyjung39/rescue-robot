#include <ros/ros.h>
#include <vector>

#include "constants/topics.h"
#include "messages/Arc.h"
#include "localization/Pose.h"
#include "localization/SimpleLocalizer.h"
// #include "sensors/Distance.h"
// #include "sensors/IMU.h"

messages::Arc arc_msg;
// float imu_yaw = 0;
// std::vector<float> high_dist_data;


void controlCommandCallback(const messages::Arc::ConstPtr &msg) {
    arc_msg = *msg;
}

// void highDistanceCallback(const sensors::Distance::ConstPtr &msg) {
//     high_dist_data = msg->data;
// }

// void imuCallback(const sensors::IMU::ConstPtr &msg) {
//     imu_yaw = msg->yaw;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_localization");
    ros::NodeHandle nh;

    ros::Subscriber control_command_sub = nh.subscribe(topics::ARC_TOPIC, 1, controlCommandCallback);
    // ros::Subscriber high_distance_sub = nh.subscribe(topics::HIGH_DIST_TOPIC, 1, highDistanceCallback);
    // ros::Subscriber imu_sub = nh.subscribe(topics::IMU_TOPIC, 1, imuCallback);
    ros::Publisher pose_pub = nh.advertise<localization::Pose>(topics::POSE_TOPIC, 1);

    float tile_time; // Time to travel one tile
    float right_turn_time;
    float left_turn_time;
    nh.param<float>("/tile_time", tile_time, 2.05);
    nh.param<float>("/right_turn_time", right_turn_time, 2.1);
    nh.param<float>("/left_turn_time", left_turn_time, 1.8);

    ros::Rate rate(10);
    SimpleLocalizer localizer(tile_time, right_turn_time, left_turn_time);
    localization::Pose pose;
    arc_msg.command_type = messages::Arc::STOP;
    while (ros::ok()) {
        ros::spinOnce();
        pose = localizer.getPoseEstimate(arc_msg);//, imu_yaw, high_dist_data);
        rate.sleep();
    }
    return 0;
}