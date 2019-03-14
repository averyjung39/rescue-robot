#include <ros/ros.h>
#include <vector>

#include "mapperception/Mapper.h"
#include "mapperception/Map.h"
#include "mapperception/MapRow.h"
#include "sensors/TimeOfFlight.h"
#include "sensors/Ultrasonic.h"
#include "localization/Pose.h"
#include "constants/topics.h"

std::vector<float> tof_sensor_data;
std::vector<float> ult_sensor_data;
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
    ros::Subscriber tof_data_sub = n.subscribe(topics::TOF_TOPIC, 1, tofSensorDataCallback);
    ros::Subscriber ult_data_sub = n.subscribe(topics::ULTRASONIC_TOPIC, 1, ultSensorDataCallback);

    ros::Publisher labeled_map_pub = n.advertise<mapperception::Map>(topics::LABELED_MAP_TOPIC, 1);
    ros::Publisher cost_map_pub = n.advertise<mapperception::Map>(topics::COST_MAP_TOPIC, 1);

    Mapper mapper;
    std::vector< std::vector<int> > cost_map;
    std::vector<mapperception::MapRow> cost_map_rows;
    mapperception::Map published_cost_map;

    int counter = 0;

    while(counter < 3) {

        ros::spinOnce();

        mapper.modifyCostMap(tof_sensor_data, robot_x, robot_y, robot_angle);
        cost_map = mapper.modifyCostMap(ult_sensor_data, robot_x, robot_y, robot_angle);

        cost_map_rows.resize(cost_map.size());
        published_cost_map.map.resize(cost_map.size());
        for(int i = 0; i < cost_map.size(); i++) {
            cost_map_rows[i].row = cost_map[i];
        }

        published_cost_map.map = cost_map_rows;

        cost_map_pub.publish(published_cost_map);

        mapper.getCostMap().print();

    }

    return 0;
}
