#include <time.h>
#include <cassert>
#include <lcmtypes/pose_xyt_t.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/particle_filter.hpp>

using namespace std;
int sampleFromWeightBar(float sample, std::vector<float> weight_bar) {
    int index = 0;
    while(sample > weight_bar[index]){
        index++;
    }
    return index;
}

ParticleFilter::ParticleFilter(int numParticles)
    : kNumParticles_(numParticles) {
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}

void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose, const OccupancyGrid& map) {
    // TODO: Implement your method for initializing the particles in the particle filter
    float map_width = map.widthInMeters();
    float map_height = map.heightInMeters();
    srand(time(NULL));
    random_device rd;
    normal_distribution<float> randomXY(0.0, min(map_width, map_height) / 20.0);
    normal_distribution<float> randomTheta(0.0, M_PI / 6.0);
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
        // printf("size of prior:     %ld\n", prior.size());
        auto proposal = computeProposalDistribution(prior);
        // printf("size of proposal:  %ld\n", proposal.size());
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // printf("size of posterior: %ld\n", posterior_.size());
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
    prior.resize(kNumParticles_);

    std::vector<float> weight_bar;
    float sum = 0.0;
    for (int i = 0; i < kNumParticles_; i++) {
        sum += posterior_[i].weight;
        weight_bar.push_back(sum);
    }
    if (abs(sum - 1.0) > 0.01) std::cout << "sum not equal to 1" << std::endl;

    srand(time(NULL));
    random_device rd;
    uniform_real_distribution<float> random(0.0, sum);
    for (int i = 0; i < kNumParticles_; i++) {
        float sample = random(rd);
        int index = sampleFromWeightBar(sample, weight_bar);
        particle_t temp = posterior_[index];
        temp.weight = 1.0 / (float)kNumParticles_;
        prior[i] = temp;
    }
    return prior;
}

std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior) {
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for (auto i : prior) {
        proposal.push_back(actionModel_.applyAction(i));
    }
    return proposal;
}

std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid& map) {
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    vector<double> logPs;
    for (auto i : proposal) {
        double logP = sensorModel_.likelihood(i, laser, map);
        logPs.push_back(logP);
    }
    double logPmax = *max_element(logPs.begin(), logPs.end());
    vector<double> shifted_P;
    double sum_shifted_P = 0.0;
    for (int i = 0; i < logPs.size(); i++) {
        logPs[i] += -logPmax;
        double temp = exp(logPs[i]);
        shifted_P.push_back(temp);
        sum_shifted_P += temp;
    }
    for (int i = 0; i < shifted_P.size(); i++) {
        particle_t temp = proposal[i];
        temp.weight = shifted_P[i] / sum_shifted_P;
        posterior.push_back(temp);
    }
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
