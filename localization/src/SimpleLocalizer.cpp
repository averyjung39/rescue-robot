#include <ros/ros.h>

#include "localization/SimpleLocalizer.h"
#include "constants/dimensions.h"
#include "messages/Arc.h"
#include "sensors/Distance.h"

SimpleLocalizer::SimpleLocalizer() {
    // Initialize pose estimate
    _current_pose.x = 7*dimensions::HALF_TILE_WIDTH_CM;
    _current_pose.y = dimensions::HALF_TILE_WIDTH_CM;
    _current_pose.theta = 90;
    _nominal_theta_deg = 90;
    _prev_control_command = messages::Arc::STOP;
    _imu_yaw_deg = 0;
    _front_distance_cm = 0;
    _back_distance_cm = 0;
    _prev_front_distance_cm = 0;
    _prev_back_distance_cm = 0;
}

localization::Pose SimpleLocalizer::getPoseEstimate(
    const messages::Arc &arc_msg,
    const float &imu_yaw,
    const std::vector<float> &high_distance_data) {
    _current_pose.is_good_reading = true;
    int control_command_type = arc_msg.command_type;    
    if (control_command_type == messages::Arc::STRAIGHT_LINE) {
        // Use high distance data to localize
        // Check if we are just starting to drive straight
        if (_prev_control_command != messages::Arc::STRAIGHT_LINE) {
            // Record distance to front and back obstacles at the start
            _front_distance_cm = high_distance_data[FRONT_INDEX];
            _back_distance_cm = high_distance_data[BACK_INDEX];

            // Calculate nominal distances (ideal distances if there are no obstacles and flat walls)
            float front_nominal_distance = _front_distance_cm, back_nominal_distance = _back_distance_cm;  
            // Get nominal distance with respect to centre of robot
            switch (_nominal_theta_deg) {
                case 0:
                    front_nominal_distance = dimensions::MAP_WIDTH - _current_pose.x;
                    back_nominal_distance = _current_pose.x;
                    break;
                case 90:
                    front_nominal_distance = dimensions::MAP_HEIGHT - _current_pose.y;
                    back_nominal_distance = _current_pose.y;
                    break;
                case 180:
                    front_nominal_distance = _current_pose.x;
                    back_nominal_distance = dimensions::MAP_WIDTH - _current_pose.x; 
                    break;
                case 270:
                    front_nominal_distance = _current_pose.y;
                    back_nominal_distance = dimensions::MAP_HEIGHT - _current_pose.y;
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
            bool valid_front_distance = isValidDistanceReading(high_distance_data[FRONT_INDEX], _prev_front_distance_cm) && _prev_front_distance_cm > high_distance_data[FRONT_INDEX];
            bool valid_back_distance = isValidDistanceReading(high_distance_data[BACK_INDEX], _prev_back_distance_cm) && _prev_back_distance_cm < high_distance_data[BACK_INDEX];
            float distance_traveled = 0;
            if (valid_front_distance && valid_back_distance) {
                // Average distance traveled
                distance_traveled = 0.5*(_prev_front_distance_cm - high_distance_data[FRONT_INDEX]) + 0.5*(high_distance_data[BACK_INDEX] - _prev_front_distance_cm);
            } else if (valid_front_distance) {
                distance_traveled = _prev_front_distance_cm - high_distance_data[FRONT_INDEX];
            } else if (valid_back_distance) {
                distance_traveled = high_distance_data[BACK_INDEX] - _prev_back_distance_cm;
            } else {
                // Both front distance and back distance are invalid
                distance_traveled = 0;
                _current_pose.is_good_reading = false;
            }

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
                _prev_front_distance_cm -= distance_traveled;
            }
            if (valid_back_distance) {
                _prev_back_distance_cm = high_distance_data[BACK_INDEX];
            } else {
                _prev_back_distance_cm += distance_traveled;
            }
        }
        // Assume theta doesn't change while driving straight
    } else if (control_command_type == messages::Arc::TURN_ON_SPOT) {
        // Check if we are just starting to turn
        if (_prev_control_command != messages::Arc::TURN_ON_SPOT) {
            // Record IMU yaw at the start of the turn
            _imu_yaw_deg = imu_yaw;
            // Update _nominal_theta_deg to the angle after we are done turning, assuming we turn 90 deg at a time
            _nominal_theta_deg = arc_msg.direction_is_right ? (_nominal_theta_deg - 90) % 360 : (_nominal_theta_deg + 90) % 360;
            _starting_angle_deg = _current_pose.theta;
        } else {
            // Update robot angle based on difference from starting angle
            _current_pose.theta = _starting_angle_deg + _imu_yaw - _imu_yaw_deg;
            if (_current_pose.theta < 0) _current_pose.theta += 360;
            if (_current_pose.theta >= 360) _current_pose.theta -= 360;
        }
        // Assume x and y don't change while turning on the spot
    } else if (control_command_type == messages::Arc::STOP) {
        // Do nothing - no pose change
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