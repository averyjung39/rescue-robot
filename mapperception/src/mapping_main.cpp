#include "ros/ros.h"
#include "Mapper.hpp"

float tof_sensor_data[5];
float ult_sensor_data[3];
float robot_x;
float robot_y;
float robot_angle;

void tofSensorDataCallback(const sensors::TimeOfFlight::ConstPtr& msg) {
    tof_sensor_data = msg->data;
}

void ultSensorDataCallback(const sensors::Ultrasonic::ConstPtr& msg) {
    ult_sensor_data = msg->data;
}

void poseCallback(const localization::Pose::ConstPtr& msg) {
    robot_x = msg->x;
    robot_y = msg->y;
    robot_angle = msg->theta;
}

int main(int argc, char **argv) {
    // create "mapping" node
    ros::init(argc, argv, "mapping");

    ros::NodeHandle n;

    // Create subscribers and publishers
    // NOTE: buffer only 1 message for now
    ros::Subscriber tof_data_sub = n.subscribe("tof_sensor_data", 1, tofSensorDataCallback);
    ros::Subscriber ult_data_sub = n.subscribe("ultrasonic_sensor_data", 1, ultSensorDataCallback);

    ros::Publisher labeled_map_pub = n.advertise<mapperception::Map>("labeled_map", 1);
    ros::Publisher terrain_map_pub = n.advertise<mapperception::Map>("terrain_map", 1);

    Mapper mapper;

    while(ros::ok()) {

        ros::spinOnce();

        mapperception::Map labeled_map;

        mapper.modifyLabeledMap(tof_sensor_data, robot_x, robot_y, robot_angle);
        labeled_map.data = mapper.modifyLabeledMap(ult_sensor_data, robot_x, robot_y, robot_angle);

        labeled_map_pub.publish(labeled_map);
    }

    return 0;
}
