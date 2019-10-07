#include <time.h>
#include <cassert>
#include <lcmtypes/pose_xyt_t.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/particle_filter.hpp>
#include <common/angle_functions.hpp>

#define RESAMPLEPORTION 0.8
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
    posteriorPose_.x = posteriorPose_.y = posteriorPose_.theta = 0.0;
}

void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose, const OccupancyGrid& map) {
    // TODO: Implement your method for initializing the particles in the particle filter
    float map_width = map.widthInMeters();
    float map_height = map.heightInMeters();
    srand(time(NULL));
    random_device rd;
    normal_distribution<float> randomXY(0.0, min(map_width, map_height) / 200.0);
    normal_distribution<float> randomTheta(0.0, M_PI / 12.0);
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
        cout << "--------------------------------------" << endl;
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

    // sample from weight bar
    uniform_real_distribution<float> random(0.0, sum);
    for (int i = 0; i < RESAMPLEPORTION * kNumParticles_; i++) {
        float sample = random(rd);
        int index = sampleFromWeightBar(sample, weight_bar);
        particle_t temp = posterior_[index];
        // if (temp.weight < 0.2) cout << "Get particle with small weight = " << temp.weight << endl;
        temp.weight = 1.0 / (float)kNumParticles_;  // gaussian???
        prior[i] = temp;
    }

    // add random particles around posteriorPose_
    normal_distribution<float> randomXY(0.0, 0.2);
    normal_distribution<float> randomTheta(0.0, M_PI / 36.0);

    for (int i = RESAMPLEPORTION * kNumParticles_; i < kNumParticles_; i++){
        particle_t temp;
        temp.parent_pose = posteriorPose_;
        temp.pose = posteriorPose_;
        temp.pose.x += randomXY(rd);
        temp.pose.y += randomXY(rd);
        temp.pose.theta = wrap_to_pi(randomTheta(rd) + temp.pose.theta);
        temp.weight = 1.0 / (float)kNumParticles_;  // gaussian???
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
        // double logP = sensorModel_.Gaussianlikelihood(i, laser, map);
        logPs.push_back(logP);
    }
    double logPmax = *max_element(logPs.begin(), logPs.end());
    cout << "log(P).max = " << logPmax << endl;
    cout << "lidar_size = " << laser.num_ranges << endl;

    vector<double> shifted_P;
    double sum_shifted_P = 0.0;
    for (size_t i = 0; i < logPs.size(); i++) {
        logPs[i] += -logPmax;
        double temp = exp(logPs[i]);
        if (temp > 0.1) cout << "shifted P = " << temp << endl;
        shifted_P.push_back(temp);
        sum_shifted_P += temp;
    }
    for (size_t i = 0; i < shifted_P.size(); i++) {
        particle_t temp = proposal[i];
        temp.weight = shifted_P[i] / sum_shifted_P;
        // if (temp.weight > 0) cout << "weight = " << temp.weight << endl;
        posterior.push_back(temp);
    }
    return posterior;
}

pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior) {  //mean pose or maximun weight pose
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    pose.utime = posterior[0].pose.utime;

    // max weight:
    // float max_weight = 0.0;
    // for (auto i : posterior) {
    //     if (max_weight < i.weight) {
    //         max_weight = i.weight;
    //         pose.x = i.pose.x;
    //         pose.y = i.pose.y;
    //         pose.theta = i.pose.theta;
    //     }
    //     if (max_weight > 0.5) {
    //         cout << "Max weight pose found = " << max_weight << endl;
    //         break;
    //     }
    // }

    // weighted mean:
    for (auto i : posterior) {
        pose.x += i.weight * i.pose.x;
        pose.y += i.weight * i.pose.y;
        pose.theta = wrap_to_pi(pose.theta + i.weight * i.pose.theta);
    }

    return pose;
}
