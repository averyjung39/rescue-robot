
#include <vector>

#include "localization/ParticleFilter.h"
#include "localization/Pose.h"
#include "sensors/Encoder.h"
#include "sensors/IMU.h"
#include "sensors/Ultrasonic.h"


ParticleFilter::ParticleFilter(const int &num_particles,
    const float &encoder_noise,
    const float &imu_noise,
    const float &ultrasonic_noise) {
    _num_particles = num_particles;
    _encoder_noise = encoder_noise;
    _imu_noise = imu_noise;
    _ultrasonic_noise = ultrasonic_noise;

    // TODO initalize vector _particles
}

localization::Pose ParticleFilter::getPoseEstimate(const sensors::Encoder &encoder_msg,
    const sensors::IMU &imu_msg,
    const sensors::Ultrasonic &ultrasonic_msg) {
    // Update all _particles based on control command
    std::vector<localization::Pose> particles_tmp = controlUpdate(encoder_msg);

    // Measurement update (get relative probabilities of each particle being a good estimate of true pose)
    std::vector<float> weights = getParticleWeights(particles_tmp, imu_msg, ultrasonic_msg);

    // Update _particles based on weights
    resample(particles_tmp, weights);

    localization::Pose avg_pose;
    // TODO generate average pose from vector of _particles
    return avg_pose;
}

std::vector<localization::Pose> ParticleFilter::controlUpdate(const sensors::Encoder &encoder_msg) const {
    std::vector<localization::Pose> particles_tmp = _particles;

    // TODO Move all particles in particles_tmp based on encoder message
    // Need to figure out a way to deal with different terrains - encoder measurements likely not reliable
    return particles_tmp;
}

std::vector<float> ParticleFilter::getParticleWeights(const std::vector<localization::Pose> &particles,
    const sensors::IMU &imu_msg,
    const sensors::Ultrasonic &ultrasonic_msg) const {
    std::vector<float> weights;

    // TODO interpret all sensor measurements and determine likelihood of each particle
    // being an accurate representation of true pose
    return weights;
}

void ParticleFilter::resample(const std::vector<localization::Pose> &particles,
    const std::vector<float> &particle_weights) {
    // TODO resample from particles and store (with replacement) in _particles
}
