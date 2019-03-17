#include <ros/ros.h>

#include "objectives/ObjectiveManager.h"
#include "objectives/ActiveObjectives.h"
#include "mapperception/Map.h"

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

    ObjectiveManager obj_manger;

    while(ros::ok()) {
        ros::spinOnce();

    }
}
