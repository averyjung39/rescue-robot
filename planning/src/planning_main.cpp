#include <ros/ros.h>

#include "mapperception/Map.h"
#include "mapperception/MapRow.h"
#include "localization/Pose.h"
#include "planning/AStarPlanner.h"
#include "planning/GoalPlanner.h"
#include "messages/Arc.h"
#include "planning/RobotPath.h"
#include "constants/topics.h"
#include "constants/labels.h"
#include "objectives/ActiveObjectives.h"
#include "std_msgs/Bool.h"

const float ANGLE_TOL_DEG = 1e-2;

mapperception::Map map_msg;
localization::Pose pose_msg;
objectives::ActiveObjectives objectives_msg;
bool controller_done;

int drive_straight_speed_r;
int drive_straight_speed_l;
int right_turn_speed_r;
int right_turn_speed_l;
int left_turn_speed_r;
int left_turn_speed_l;

void labelMapCallback(const mapperception::Map::ConstPtr &msg) {
    map_msg = *msg;
}

void poseCallback(const localization::Pose::ConstPtr &msg) {
    pose_msg = *msg;
}

void objectivesCallback(const objectives::ActiveObjectives::ConstPtr &msg) {
    objectives_msg = *msg;
}

void controllerDoneCallback(const std_msgs::Bool::ConstPtr &msg) {
    controller_done = msg->data;
}

int robotDirectionToAngle(const RobotDirections &direction) {
    switch (direction) {
        case ROBOT_DIRECTION_UP: return 90;
        case ROBOT_DIRECTION_DOWN: return 270;
        case ROBOT_DIRECTION_LEFT: return 180;
        case ROBOT_DIRECTION_RIGHT: return 0;
        default: return -1;
    }
}

int getAngleToNextCell(const std::pair<int, int> &curr_indices,
    const std::pair<int, int> &goal_indices) {
    int i_diff = goal_indices.first - curr_indices.first;
    int j_diff = goal_indices.second - curr_indices.second;

    if (i_diff == 1 && j_diff == 0) return 270;
    else if (i_diff == -1 && j_diff == 0) return 90;
    else if (i_diff == 0 && j_diff == 1) return 0;
    else if (i_diff == 0 && j_diff == -1) return 180;

    return -1;
}

// Returns proper value of direction_is_right
// thetas from 0 to 360
bool getTurningDirection(const float &robot_theta, const float &destination_theta) {
    if (destination_theta > robot_theta) {
        return destination_theta - robot_theta > 180;
    } else {
        return robot_theta - destination_theta < 180;
    }
}

messages::Arc pathToControlCommand(const RobotPath &path, const float &robot_theta, const GoalPose &goal) {
    messages::Arc control_cmd;
    control_cmd.num_tiles = 1;
    if (path.size() == 0) {
        // No A* solution was found
        control_cmd.command_type = messages::Arc::STOP;
        control_cmd.direction_is_right = true; // Doesn't matter
        control_cmd.speed_r = control_cmd.speed_l = 0;
        return control_cmd;
    }

    if (path[0].first == goal.i && path[0].second == goal.j) {
        // We are at the right position
        float goal_angle = robotDirectionToAngle(goal.direction);
        if (goal.direction == ROBOT_DIRECTION_ANY || fabs(goal_angle - robot_theta) < ANGLE_TOL_DEG) {
            // We are at the goal (right position and orientation)
            control_cmd.command_type = messages::Arc::STOP;
            control_cmd.direction_is_right = true;
            control_cmd.speed_r = control_cmd.speed_l = 0;
        } else {
            // We need to turn on the spot (right position, wrong orientation)
            control_cmd.command_type = messages::Arc::TURN_ON_SPOT;
            control_cmd.direction_is_right = getTurningDirection(robot_theta, goal_angle);
            control_cmd.speed_r = control_cmd.direction_is_right ? right_turn_speed_r : left_turn_speed_r;
            control_cmd.speed_l = control_cmd.direction_is_right ? right_turn_speed_l : left_turn_speed_l;
        }
    } else {
        // We are not at the right position
        int destination_angle = getAngleToNextCell(path[0], path[1]);
        if (fabs(destination_angle - robot_theta) < ANGLE_TOL_DEG) {
            // Continue going straight
            control_cmd.command_type = messages::Arc::STRAIGHT_LINE;
            control_cmd.direction_is_right = true; // Forward
            control_cmd.speed_r = drive_straight_speed_r;
            control_cmd.speed_l = drive_straight_speed_l;

            int i_diff = path[1].first - path[0].first;
            int j_diff = path[1].second - path[1].second;
            int num_tiles = 1;
            int index = 1;
            int i_diff_original = i_diff;
            int j_diff_original = j_diff;
            while (index < path.size()-1 && i_diff == i_diff_original && j_diff == j_diff_original) {
                i_diff = path[index+1].first - path[index].first;
                j_diff = path[index+1].second - path[index].second;
                ++num_tiles;
                ++index;
            }
            control_cmd.num_tiles = num_tiles;
        } else {
            // Turn on the spot
            control_cmd.command_type = messages::Arc::TURN_ON_SPOT;
            control_cmd.direction_is_right = getTurningDirection(robot_theta, destination_angle);
            control_cmd.speed_r = control_cmd.direction_is_right ? right_turn_speed_r : left_turn_speed_r;
            control_cmd.speed_l = control_cmd.direction_is_right ? right_turn_speed_l : left_turn_speed_l;        
        }
    }

    return control_cmd;
}

bool shouldScan(const mapperception::Map &map, const localization::Pose &pose) {
    int robot_i = map.robot_i;
    int robot_j = map.robot_j;
    int robot_theta = pose.theta;

    int i_in_front, j_in_front;

    if (robot_theta > 80 && robot_theta < 100) {
        i_in_front = robot_i - 1;
        j_in_front = robot_j;
    } else if (robot_theta > 170 && robot_theta < 190) {
        i_in_front = robot_i;
        j_in_front = robot_j - 1;
    } else if (robot_theta > 260 && robot_theta < 280) {
        i_in_front = robot_i + 1;
        j_in_front = robot_j;
    } else if (robot_theta > 350 || robot_theta < 10) {
        i_in_front = robot_i;
        j_in_front = robot_j + 1; 
    } else {
        return false;
    }
    return (i_in_front >= 0 && i_in_front <= 5 &&
        j_in_front >= 0 && j_in_front <= 5 &&
        (map.map[i_in_front].row[j_in_front] == labels::TALL_OBJECT || map.map[i_in_front].row[j_in_front] == labels::OBJECT));
}

messages::Arc leftTurn() {
    messages::Arc arc_cmd;
    arc_cmd.command_type = messages::Arc::TURN_ON_SPOT;
    arc_cmd.direction_is_right = false;
    arc_cmd.speed_r = left_turn_speed_r;
    arc_cmd.speed_l = left_turn_speed_l;
    arc_cmd.num_tiles = 1;
    return arc_cmd;
}

messages::Arc rightTurn() {
    messages::Arc arc_cmd;
    arc_cmd.command_type = messages::Arc::TURN_ON_SPOT;
    arc_cmd.direction_is_right = true;
    arc_cmd.speed_r = right_turn_speed_r;
    arc_cmd.speed_l = right_turn_speed_l;
    arc_cmd.num_tiles = 1;
    return arc_cmd;
}

messages::Arc stop() {
    messages::Arc arc_cmd;
    arc_cmd.command_type = messages::Arc::STOP;
    arc_cmd.direction_is_right = true;
    arc_cmd.speed_r = 0;
    arc_cmd.speed_l = 0;
    arc_cmd.num_tiles = 1;
    return arc_cmd;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe(topics::LABEL_MAP_TOPIC, 1, labelMapCallback);
    ros::Subscriber pose_sub = nh.subscribe(topics::POSE_TOPIC, 1, poseCallback);
    ros::Subscriber obj_sub = nh.subscribe(topics::OBJECTIVE_TOPIC, 1, objectivesCallback);
    ros::Subscriber controller_done_sub = nh.subscribe(topics::CONTROLLER_DONE, 1, controllerDoneCallback);
    ros::Publisher arc_pub = nh.advertise<messages::Arc>(topics::ARC_TOPIC, 1);
    ros::Publisher scanning_pub = nh.advertise<std_msgs::Bool>(topics::SCANNING, 1);

    nh.param<int>("/drive_straight_speed_r", drive_straight_speed_r, 120);
    nh.param<int>("/drive_straight_speed_l", drive_straight_speed_l, 110);
    nh.param<int>("/right_turn_speed_r", right_turn_speed_r, 85);
    nh.param<int>("/right_turn_speed_l", right_turn_speed_l, 110);
    nh.param<int>("/left_turn_speed_r", left_turn_speed_r, 110);
    nh.param<int>("/left_turn_speed_l", left_turn_speed_l, 90);

    GoalPlanner goal_planner;
    AStarPlanner a_star_planner;
    GoalPose goal_pose;
    messages::Arc arc_cmd;
    std_msgs::Bool scanning_msg;
    bool scanning = false;
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        controller_done = false;
        if (!scanning) {
            goal_pose = goal_planner.getGoal(pose_msg, map_msg, objectives_msg);
            std::pair<int, int> start_pos = std::make_pair(map_msg.robot_i, map_msg.robot_j);
            std::pair<int, int> end_pos = std::make_pair(goal_pose.i, goal_pose.j);
            RobotPath path = a_star_planner.planPath(map_msg, start_pos, end_pos);
            arc_cmd = pathToControlCommand(path, pose_msg.theta, goal_pose);
            arc_pub.publish(arc_cmd);
            while (!controller_done) {
                ros::spinOnce();
            }
            arc_cmd = stop();
            arc_pub.publish(arc_cmd);
            while (!controller_done) {
                ros::spinOnce();
            }

            scanning = shouldScan(map_msg, pose_msg);
            scanning_msg.data = scanning;
            scanning_pub.publish(scanning_msg);
        } else {
            // Turn left
            arc_cmd = leftTurn();
            arc_pub.publish(arc_cmd);
            while(!controller_done) {
                ros::spinOnce();
            }
            arc_cmd = stop();
            arc_pub.publish(arc_cmd);
            while(!controller_done) {
                ros::spinOnce();
            }

            // Turn right twice
            arc_cmd = rightTurn();
            arc_pub.publish(arc_cmd);
            while(!controller_done) {
                ros::spinOnce();
            }
            arc_cmd = stop();
            arc_pub.publish(arc_cmd);
            while(!controller_done) {
                ros::spinOnce();
            }
            arc_cmd = rightTurn();
            arc_pub.publish(arc_cmd);
            while(!controller_done) {
                ros::spinOnce();
            }
            arc_cmd = stop();
            arc_pub.publish(arc_cmd);
            while(!controller_done) {
                ros::spinOnce();
            }

            // Turn left
            arc_cmd = leftTurn();
            arc_pub.publish(arc_cmd);
            while(!controller_done) {
                ros::spinOnce();
            }
            arc_cmd = stop();
            arc_pub.publish(arc_cmd);
            while(!controller_done) {
                ros::spinOnce();
            }
            scanning = false;
            scanning_msg.data = scanning;
            scanning_pub.publish(scanning_msg);
        }
        rate.sleep();
    }

    return 0;
}