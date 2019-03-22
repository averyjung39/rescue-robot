#include <ros/ros.h>
#include <vector>

#include "mapperception/Mapper.h"
#include "mapperception/Map.h"
#include "sensors/Distance.h"
#include "sensors/Photodiode.h"
#include "localization/Pose.h"
#include "std_msgs/Bool.h"
#include "constants/topics.h"
#include "messages/Arc.h"

std::vector<float> low_dists;
std::vector<float> high_dists;
std::vector<int> photodiode_data;
bool hall_effect_data;
bool scanning;
int arc_cmd;

float robot_x;
float robot_y;
float robot_angle;

void lowDistDataCallback(const sensors::Distance::ConstPtr& msg) {
    low_dists = msg->data;
}

void highDistDataCallback(const sensors::Distance::ConstPtr& msg) {
    high_dists = msg->data;
}

void flameSensorDataCallback(const sensors::Photodiode::ConstPtr& msg) {
    photodiode_data = msg->data;
}

void hallEffectDataCallback(const std_msgs::Bool::ConstPtr& msg) {
    hall_effect_data = msg->data;
}

void poseCallback(const localization::Pose::ConstPtr& msg) {
    robot_x = msg->x;
    robot_y = msg->y;
    robot_angle = msg->theta;
}

void scanningCallback(const std_msgs::Bool::ConstPtr& msg) {
    scanning = msg->data;
}

void arcCallback(const messages::Arc::ConstPtr &msg) {
    arc_cmd = msg->command_type;
}

int main(int argc, char **argv) {
    // create "mapping" node
    ros::init(argc, argv, "mapping");

    ros::NodeHandle nh;

    // Create subscribers and publishers
    // NOTE: buffer only 1 message for now
    ros::Subscriber low_dist_sub = nh.subscribe(topics::LOW_DIST_TOPIC, 1, lowDistDataCallback);
    ros::Subscriber high_dist_sub = nh.subscribe(topics::HIGH_DIST_TOPIC, 1, highDistDataCallback);
    ros::Subscriber photodiode_sub = nh.subscribe(topics::PHOTODIODE_TOPIC, 1, flameSensorDataCallback);
    ros::Subscriber hall_effect_sub = nh.subscribe(topics::HALL_EFFECT_TOPIC, 1, hallEffectDataCallback);
    ros::Subscriber pose_sub = nh.subscribe(topics::POSE_TOPIC, 1, poseCallback);
    ros::Subscriber scanning = nh.subscribe(topics::SCANNING, 1, scanningCallback);
    ros::Publisher label_map_publisher = nh.advertise<mapperception::Map>(topics::LABEL_MAP_TOPIC, 1);

    // TODO: Integrate button for figuring out orientation;
    int orientation = 1;

    Mapper mapper = Mapper(orientation);

    std::vector< std::vector<int> > label_map;
    std::vector<mapperception::MapRow> label_map_rows;
    mapperception::Map published_label_map;

    bool big_house_detected = false, fire_detected = false;

    while(ros::ok()) {

        ros::spinOnce();

        mapper.setRobotPose(robot_x, robot_y, robot_angle);
        // Don't map if the robot is turning
        if (arc_cmd != messages::Arc::TURN_ON_SPOT) {
            if (scanning) {
                if (!fire_detected) fire_detected = mapper.detectFire(photodiode_data);
                if ((high_dists[2] != sensors::Distance::INVALID_SENSOR_DATA ||
                    high_dists[3] != sensors::Distance::INVALID_SENSOR_DATA) &&
                    !big_house_detected)
                {
                    big_house_detected = mapper.detectHouses(high_dists[2], high_dists[3]);
                }
            } else {
                mapper.updateLabelMapWithScanningResults(big_house_detected, fire_detected);
                mapper.modifyLabelMapWithPhotodiode(photodiode_data);
                // Try detecting what the terrain is right in front the robot
                mapper.detectMagnet(hall_effect_data);
                mapper.modifyLabelMapWithDists(low_dists, false);
                mapper.modifyLabelMapWithDists(high_dists, true);
            }
        }

        label_map = mapper.getLabelMap().getMap();
        label_map_rows.resize(label_map.size());
        published_label_map.map.resize(label_map.size());

        for(int i = 0; i < label_map.size(); i++) {
            label_map_rows[i].row = label_map[i];
        }

        published_label_map.map = label_map_rows;
        label_map_publisher.publish(published_label_map);

        mapper.getLabelMap().print();

    }

    return 0;
}
