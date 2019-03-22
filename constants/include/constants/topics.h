#ifndef TOPICS
#define TOPICS

namespace topics {
    // Sensors
    const char *LOW_DIST_TOPIC = "low_dist_data";
    const char *HIGH_DIST_TOPIC = "high_dist_data";
    const char *ULTRASONIC_TOPIC = "ultrasonic_sensor_data";
    const char *PHOTODIODE_TOPIC = "photodiode_sensor_data";
    const char *HALL_EFFECT_TOPIC = "hall_effect_sensor_data";
    const char *ENCODER_TOPIC = "encoder_data";
    const char *IMU_TOPIC = "imu_sensor_data";

    // Path-Planning
    const char *ARC_TOPIC = "arc_data";
    const char *SCANNING = "scan_topic";

    // Mapperception
    const char *LABEL_MAP_TOPIC = "label_map";

    // Localization
    const char *POSE_TOPIC = "pose";

    // Objectives
    const char *OBJECTIVE_TOPIC = "active_objectives";

    //Controller
    const char *CONTROLLER_DONE = "controller_done";

    // Construction Check Demos
    const char *DRIVE_DEMO_COMPLETE_TOPIC = "drive_demo_complete";
    const char *TURN_DEMO_COMPLETE_TOPIC = "turn_demo_complete";
    const char *TOF_DEMO_COMPLETE_TOPIC = "tof_demo_complete";
}
#endif // TOPICS
