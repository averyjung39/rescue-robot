#include <ros/ros.h>
#include <vector>
#include "objectives/ObjectiveManager.h"
#include "objectives/ActiveObjectives.h"
#include "mapperception/Map.h"
#include "constants/topics.h"

// current robot indices
int robot_i;
int robot_j;

void labelMapCallback(const mapperception::Map::ConstPtr& msg) {
    robot_i = msg->robot_i;
    robot_j = msg->robot_j;
}

int main(int argc, char** argv) {
    // Create objectives node
    ros::init(argc, argv, "objectives");

    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe(topics::LABEL_MAP_TOPIC, 1, labelMapCallback);
    ros::Publisher active_objs_pub = nh.advertise<objectives::ActiveObjectives>(topics::OBJECTIVE_TOPIC, 1);

    ObjectiveManager obj_manager = ObjectiveManager(nh, topics::OBJECT_LOCATION_SERVICE);
    std::vector<bool> active_objs;
    objectives::ActiveObjectives active_objs_msg;
    active_objs_msg.active_objectives.resize(5);

    while(ros::ok()) {
        ros::spinOnce();

        active_objs = obj_manager.activateObjectives(robot_i, robot_j);
        for(int i = 0; i < active_objs.size(); i++) {
            active_objs_msg.active_objectives[i] = active_objs[i];
        }
        active_objs_pub.publish(active_objs_msg);
    }
}
