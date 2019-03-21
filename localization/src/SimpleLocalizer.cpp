#include <ros/ros.h>

#include "localization/SimpleLocalizer.h"
#include "constants/dimensions.h"
#include "messages/Arc.h"
#include "sensors/Distance.h"

SimpleLocalizer::SimpleLocalizer() {
    SimpleLocalizer(2.05, 2.1, 1.8);
}

SimpleLocalizer::SimpleLocalizer(const float &tile_time,
    const float &right_turn_time,
    const float &left_turn_time) {
    // Initialize pose estimate
    _current_pose.x = 7*dimensions::HALF_TILE_WIDTH_CM;
    _current_pose.y = dimensions::HALF_TILE_WIDTH_CM;
    _current_pose.theta = 90;
    _nominal_theta_deg = 90;
    _prev_control_command = messages::Arc::STOP;
    _starting_imu_yaw = 0;
    _front_distance_cm = 0;
    _back_distance_cm = 0;
    _prev_front_distance_cm = 0;
    _prev_back_distance_cm = 0;
    _straight_line_speed = 0;
    _nominal_x_cm = _current_pose.x;
    _nominal_y_cm = _current_pose.y;
    _angular_speed = 0;
    _tile_time = tile_time;
    _right_turn_time = right_turn_time;
    _left_turn_time = left_turn_time;
}

localization::Pose SimpleLocalizer::getPoseEstimate(
    const messages::Arc &arc_msg,
    const float &imu_yaw,
    const std::vector<float> &high_distance_data) {
    _current_pose.is_good_reading = true;
    int control_command_type = arc_msg.command_type;    
    if (control_command_type == messages::Arc::STRAIGHT_LINE) {
        // Check if we are just starting to drive straight
        if (_prev_control_command != messages::Arc::STRAIGHT_LINE) {
            // Get nominal straight line speed based on hard-coded times
            _straight_line_speed = dimensions::TILE_WIDTH_CM / _tile_time;
            _prev_time = ros::Time::now().toSec();

            // Record ToF distance to front and back obstacles at the start
            _front_distance_cm = high_distance_data[FRONT_INDEX];
            _back_distance_cm = high_distance_data[BACK_INDEX];

            // Calculate nominal distances (ideal distances if there are no obstacles and flat walls)
            float front_nominal_distance = _front_distance_cm, back_nominal_distance = _back_distance_cm;  
            // Get nominal distance with respect to centre of robot
            switch (_nominal_theta_deg) {
                case 0:
                    front_nominal_distance = dimensions::MAP_WIDTH - _current_pose.x;
                    back_nominal_distance = _current_pose.x;
                    _nominal_x_cm = _current_pose.x + dimensions::TILE_WIDTH_CM * arc_msg.num_tiles;
                    break;
                case 90:
                    front_nominal_distance = dimensions::MAP_HEIGHT - _current_pose.y;
                    back_nominal_distance = _current_pose.y;
                    _nominal_y_cm = _current_pose.y + dimensions::TILE_WIDTH_CM * arc_msg.num_tiles;
                    break;
                case 180:
                    front_nominal_distance = _current_pose.x;
                    back_nominal_distance = dimensions::MAP_WIDTH - _current_pose.x;
                    _nominal_x_cm = _current_pose.x - dimensions::TILE_WIDTH_CM * arc_msg.num_tiles;
                    break;
                case 270:
                    front_nominal_distance = _current_pose.y;
                    back_nominal_distance = dimensions::MAP_HEIGHT - _current_pose.y;
                    _nominal_y_cm = _current_pose.y - dimensions::TILE_WIDTH_CM * arc_msg.num_tiles;
                    break;
                default:
                    ROS_ERROR("Unhandled nominal theta: %d", _nominal_theta_deg);
                    _current_pose.is_good_reading = false;
                    break;
            }
            // Subtract sensor offset to get nominal distance with respect to sensor location
            front_nominal_distance -= dimensions::TOP_TOF_FRONT_X_OFFSET;
            back_nominal_distance -= dimensions::TOP_TOF_OFFSET;

            // Use nominal values if measured sensor data is invalid
            if (_front_distance_cm == sensors::Distance::INVALID_SENSOR_DATA) {
                _front_distance_cm = front_nominal_distance;
            }
            if (_back_distance_cm == sensors::Distance::INVALID_SENSOR_DATA) {
                _back_distance_cm = back_nominal_distance;
            }
            _prev_front_distance_cm = _front_distance_cm;
            _prev_back_distance_cm = _back_distance_cm;
        } else {
            // Get distance traveled based on hard-coded times
            float dt = ros::Time::now().toSec() - _prev_time;
            float distance_traveled_hardcode = _straight_line_speed * dt;
            _prev_time = ros::Time::now().toSec();

            // Get distance traveled based on ToFs
            bool valid_front_distance = isValidDistanceReading(high_distance_data[FRONT_INDEX], _prev_front_distance_cm) && _prev_front_distance_cm > high_distance_data[FRONT_INDEX];
            bool valid_back_distance = isValidDistanceReading(high_distance_data[BACK_INDEX], _prev_back_distance_cm) && _prev_back_distance_cm < high_distance_data[BACK_INDEX];
            float distance_traveled_tof = 0;
            if (valid_front_distance && valid_back_distance) {
                // Average distance traveled
                distance_traveled_tof = 0.5*(_prev_front_distance_cm - high_distance_data[FRONT_INDEX]) + 0.5*(high_distance_data[BACK_INDEX] - _prev_front_distance_cm);
            } else if (valid_front_distance) {
                distance_traveled_tof = _prev_front_distance_cm - high_distance_data[FRONT_INDEX];
            } else if (valid_back_distance) {
                distance_traveled_tof = high_distance_data[BACK_INDEX] - _prev_back_distance_cm;
            } else {
                // Both front distance and back distance are invalid
                distance_traveled_tof = 0;
                _current_pose.is_good_reading = false;
            }

            // Fuse distance calculations
            // Weightings are arbitrary for now
            float distance_traveled = 0.7*distance_traveled_hardcode + 0.3*distance_traveled_tof;

            // Update pose based on distance traveled
            switch (_nominal_theta_deg) {
                case 0:
                    _current_pose.x += distance_traveled;
                    break;
                case 90:
                    _current_pose.y += distance_traveled;
                    break;
                case 180:
                    _current_pose.x -= distance_traveled;
                    break;
                case 270:
                    _current_pose.y -= distance_traveled;
                    break;
                default:
                    ROS_ERROR("Unhandled nominal theta: %d", _nominal_theta_deg);
                    _current_pose.is_good_reading = false;
                    break;
            }

            if (valid_front_distance) {
                _prev_front_distance_cm = high_distance_data[FRONT_INDEX];
            } else {
                _prev_front_distance_cm -= distance_traveled_tof;
            }
            if (valid_back_distance) {
                _prev_back_distance_cm = high_distance_data[BACK_INDEX];
            } else {
                _prev_back_distance_cm += distance_traveled_tof;
            }
        }
        // Assume theta doesn't change while driving straight
    } else if (control_command_type == messages::Arc::TURN_ON_SPOT) {
        // Check if we are just starting to turn
        if (_prev_control_command != messages::Arc::TURN_ON_SPOT) {
            // Get angular speed given hard coded values
            _angular_speed = arc_msg.direction_is_right ? 90.0 / _right_turn_time : 90.0 / _left_turn_time;
            _prev_time = ros::Time::now().toSec();

            // Record IMU yaw at the start of the turn
            _starting_imu_yaw = imu_yaw;
            // Update _nominal_theta_deg to the angle after we are done turning, assuming we turn 90 deg at a time
            _nominal_theta_deg = arc_msg.direction_is_right ? (_nominal_theta_deg - 90) % 360 : (_nominal_theta_deg + 90) % 360;
            _starting_angle_deg = _current_pose.theta;
        } else {
            // Get angle from hard-coded times
            float dt = ros::Time::now().toSec() - _prev_time;
            float angle_change_hardcode = _angular_speed * dt;
            _prev_time = ros::Time::now().toSec();

            // Update IMU angle based on difference from starting angle
            float angle_change_imu = _starting_angle_deg + imu_yaw - _starting_imu_yaw;
            _current_pose.theta += 0.7*angle_change_hardcode + 0.3*angle_change_imu;

            if (_current_pose.theta < 0) _current_pose.theta += 360;
            if (_current_pose.theta >= 360) _current_pose.theta -= 360;
        }
        // Assume x and y don't change while turning on the spot
    } else if (control_command_type == messages::Arc::STOP) {
        if (_prev_control_command == messages::Arc::STRAIGHT_LINE) {
            // If we were driving straight, set x and y to nominal values
            _current_pose.x = _nominal_x_cm;
            _current_pose.y = _nominal_y_cm;
        } else if (_prev_control_command == messages::Arc::TURN_ON_SPOT) {
            // If we were turning, set theta to nominal value
            _current_pose.theta = _nominal_theta_deg;
        }
    } else {
        ROS_ERROR("Unknown control_command_type: %d", control_command_type);
    }
    _prev_control_command = control_command_type;
    return _current_pose;
}

bool SimpleLocalizer::isValidDistanceReading(const float &distance,
    const float &prev_distance) const {
    return distance != sensors::Distance::INVALID_SENSOR_DATA &&
        abs(distance - prev_distance) < MAX_STRAIGHT_LINE_DISTANCE_DEVIATION_CM;
}