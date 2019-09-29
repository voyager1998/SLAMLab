#include <cassert>
#include <lcmtypes/pose_xyt_t.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/particle_filter.hpp>
#include <time.h>

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
    uniform_real_distribution<float> randomXY(-min(map_width, map_height) / 5.0, min(map_width, map_height) / 5.0);
    uniform_real_distribution<float> randomTheta(-M_PI, M_PI);//???
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

std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void) {
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    std::vector<particle_t> prior;
    return prior;
}

std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior) {
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

pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior) {
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    return pose;
}
