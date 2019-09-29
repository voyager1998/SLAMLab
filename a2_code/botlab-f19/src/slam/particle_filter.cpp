#include <time.h>
#include <cassert>
#include <lcmtypes/pose_xyt_t.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/particle_filter.hpp>

using namespace std;

ParticleFilter::ParticleFilter(int numParticles)
    : kNumParticles_(numParticles) {
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}

void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose, const OccupancyGrid& map) {
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    float map_width = map.widthInMeters();
    float map_height = map.heightInMeters();
    srand(time(NULL));
    random_device rd;
    normal_distribution<float> randomXY(0.0, min(map_width, map_height) / 20.0);
    normal_distribution<float> randomTheta(0.0, M_PI / 6.0);  //???
    for (int i = 0; i < kNumParticles_; i++) {
        posterior_[i].pose.x = pose.x + randomXY(rd);
        posterior_[i].pose.y = pose.y + randomXY(rd);
        posterior_[i].pose.theta = pose.theta + randomTheta(rd);
        posterior_[i].weight = 1.0 / (float)kNumParticles_;
    }
}

pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t& odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid& map) {
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if (hasRobotMoved) {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

pose_xyt_t ParticleFilter::poseEstimate(void) const {
    return posteriorPose_;
}

particles_t ParticleFilter::particles(void) const {
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}

std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void) {  //action model update
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    std::vector<particle_t> prior;
    return prior;
}

std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior) {  //weight update based on sensormodel
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    return proposal;
}

std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid& map) {
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    return posterior;
}

pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior) {  //mean pose or maximun weight pose
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    float max_weight = 0.0;
    for (auto i : posterior_) {
        if (max_weight < i.weight) {
            max_weight = i.weight;
            pose.x = i.pose.x;
            pose.y = i.pose.y;
            pose.theta = i.pose.theta;
        }
    }
    return pose;
}
