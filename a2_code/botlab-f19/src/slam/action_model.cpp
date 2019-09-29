#include <time.h>
#include <cassert>
#include <cmath>
#include <common/angle_functions.hpp>
#include <iostream>
#include <lcmtypes/particle_t.hpp>
#include <slam/action_model.hpp>
using namespace std;

ActionModel::ActionModel(void) {
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    previous_odo.x = 0.0f;
    previous_odo.y = 0.0f;
    previous_odo.theta = 0.0f;
}

bool ActionModel::updateAction(const pose_xyt_t& odometry) {
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (previous_odo.x != odometry.x || previous_odo.y != odometry.y || previous_odo.theta != odometry.theta) {
        de_odo.x = odometry.x - previous_odo.x;
        de_odo.y = odometry.y - previous_odo.y;
        de_odo.theta = odometry.theta - previous_odo.theta;
        previous_odo.utime = odometry.utime;
        previous_odo.x = odometry.x;
        previous_odo.y = odometry.y;
        previous_odo.theta = odometry.theta;
        return true;
    }
    return false;
}

particle_t ActionModel::applyAction(const particle_t& sample) {
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_sample;
    srand(time(NULL));
    random_device rd;
    normal_distribution<float> randomXY(0.0, 0.2);
    normal_distribution<float> randomTheta(0.0, M_PI / 6.0);
    new_sample.parent_pose.utime = sample.pose.utime;
    new_sample.parent_pose.x = sample.pose.x;
    new_sample.parent_pose.y = sample.pose.y;
    new_sample.parent_pose.theta = sample.pose.theta;
    // new_sample.pose.utime = sample.pose.utime;
    float orient = fmod(sample.pose.theta + de_odo.theta + randomTheta(rd), 2 * M_PI);
    float dist = sqrt(de_odo.x * de_odo.x + de_odo.y * de_odo.y);
    new_sample.pose.x = sample.pose.x + (cos(orient) * dist);
    new_sample.pose.y = sample.pose.y + (sin(orient) * dist);
    new_sample.pose.theta = orient;
    new_sample.weight = sample.weight;
    return new_sample;
}
