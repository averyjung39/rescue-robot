#include <ros/ros.h>
#include <vector>

#include "mapperception/Mapper.h"
#include "mapperception/Map.h"
#include "sensors/Distance.h"
#include "sensors/Ultrasonic.h"
#include "localization/Pose.h"
#include "constants/topics.h"

std::vector<float> low_dists;
std::vector<float> high_dists;
std::vector<float> ult_sensor_data;
float robot_x;
float robot_y;
float robot_angle;

void lowDistDataCallback(const sensors::Distance::ConstPtr& msg) {
    low_dists = msg->data;
}

void highDistDataCallback(const sensors::Distance::ConstPtr& msg) {
    high_dists = msg->data;
}

void ultSensorDataCallback(const sensors::Distance::ConstPtr& msg) {
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

    ros::NodeHandle nh;

    // Create subscribers and publishers
    // NOTE: buffer only 1 message for now
    ros::Subscriber low_dist_sub = nh.subscribe(topics::LOW_DIST_TOPIC, 1, lowDistDataCallback);
    ros::Subscriber high_dist_sub = nh.subscribe(topics::HIGH_DIST_TOPIC, 1, highDistDataCallback);
    ros::Subscriber ult_data_sub = nh.subscribe(topics::ULTRASONIC_TOPIC, 1, ultSensorDataCallback);
    ros::Subscriber pose_sub = nh.subscribe(topics::POSE_TOPIC, 1, poseCallback);

    ros::Publisher label_map_publisher = nh.advertise<mapperception::Map>(topics::LABEL_MAP_TOPIC, 1);

    Mapper mapper;
    std::vector< std::vector<int> > label_map;
    std::vector<mapperception::MapRow> label_map_rows;
    mapperception::Map published_label_map;

    int counter = 0;

    while(counter < 3) {

        ros::spinOnce();

        mapper.modifyLabelMapWithDists(low_dists, robot_x, robot_y, robot_angle);
        label_map = mapper.getLabelMap().getMap();
        label_map_rows.resize(label_map.size());
        published_label_map.map.resize(label_map.size());

        for(int i = 0; i < label_map.size(); i++) {
            label_map_rows[i].row = label_map[i];
        }

        published_label_map.map = label_map_rows;
        label_map_publisher.publish(published_label_map);

        mapper.getLabelMap().print();

        counter++;

    }

    return 0;
}
