#include <ros/ros.h>
#include <vector>
#include <std_msgs/Bool.h>
#include "objectives/ObjectiveManager.h"
#include "objectives/ActiveObjectives.h"
#include "mapperception/Map.h"
#include "constants/topics.h"

// current robot indices
int robot_i;
int robot_j;
std::vector<mapperception::MapRow> map_rows;

void labelMapCallback(const mapperception::Map::ConstPtr& msg) {
    map_rows = msg->map;
    robot_i = msg->robot_i;
    robot_j = msg->robot_j;
}

int main(int argc, char** argv) {
    // Create objectives node
    ros::init(argc, argv, "objectives");

    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe(topics::LABEL_MAP_TOPIC, 1, labelMapCallback);
    ros::Publisher active_objs_pub = nh.advertise<objectives::ActiveObjectives>(topics::OBJECTIVE_TOPIC, 1);
    ros::Publisher fan_inactive_pub = nh.advertise<std_msgs::Bool>(topics::FAN_TOPIC, 1);

    ObjectiveManager obj_manager = ObjectiveManager();
    std::vector< std::vector<int> > label_map;
    label_map.resize(6);
    std::vector<bool> active_objs;
    objectives::ActiveObjectives active_objs_msg;
    active_objs_msg.active_objectives.resize(6);

    while(ros::ok()) {
        ros::spinOnce();
        for(int i = 0; i < map_rows.size(); i++) {
            label_map[i] = map_rows[i].row;
        }
        active_objs = obj_manager.activateObjectives(robot_i, robot_j, label_map);
        // Fire has been turned off
        fan_inactive_pub.publish(!active_objs[0]);
        for(int i = 0; i < active_objs.size(); i++) {
            active_objs_msg.active_objectives[i] = active_objs[i];
        }
        active_objs_pub.publish(active_objs_msg);
    }
}
