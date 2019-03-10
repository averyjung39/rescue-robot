#ifndef PARTICLE_FILTER
#define PARTICLE_FILTER

#include <vector>

#include "localization/Pose.h"
#include "sensors/Encoder.h"
#include "sensors/IMU.h"
#include "sensors/Ultrasonic.h"

class ParticleFilter {
public:
    ParticleFilter(const int &num_particles,
        const float &encoder_noise,
        const float &imu_noise,
        const float &ultrasonic_noise);
    localization::Pose getPoseEstimate(const sensors::Encoder &encoder_msg,
        const sensors::IMU &imu_msg,
        const sensors::Ultrasonic &ultrasonic_msg);
private:
    int _num_particles;
    float _encoder_noise;
    float _imu_noise;
    float _ultrasonic_noise;
    std::vector<localization::Pose> _particles;

    std::vector<localization::Pose> controlUpdate(const sensors::Encoder &encoder_msg) const;
    std::vector<float> getParticleWeights(const std::vector<localization::Pose> &particles,
        const sensors::IMU &imu_msg,
        const sensors::Ultrasonic &ultrasonic_msg) const;
    void resample(const std::vector<localization::Pose> &particles,
        const std::vector<float> &particle_weights);
};

#endif  // PARTICLE_FILTER