#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    previous_odo.x = 0.0f;
    previous_odo.y = 0.0f;
    previous_odo.theta = 0.0f;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
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


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_sample;
    new_sample.parent_pose.utime = sample.pose.utime;
    new_sample.parent_pose.x = sample.pose.x;
    new_sample.parent_pose.y = sample.pose.y;
    new_sample.parent_pose.theta = sample.pose.theta;
    // new_sample.pose.utime = sample.pose.utime;
    new_sample.pose.x = sample.pose.x + de_odo.x;
    new_sample.pose.y = sample.pose.y + de_odo.y;
    new_sample.pose.theta = sample.pose.theta + de_odo.theta;
    return new_sample;
}
