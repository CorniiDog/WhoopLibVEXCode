/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       KalmanFilter.cpp                                          */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Kalman Filter for Odometry                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/calculators/KalmanFilter.hpp"


KalmanFilter::KalmanFilter(int n) : capacity(n), estimate(), velocity() {}

void KalmanFilter::addMeasurement(const Pose& newPose){
    // Add new measurement to the history
    if (history.size() >= capacity) {
        history.pop_front();
    }
    history.push_back(newPose);

    // Compute the rolling average for the pose and velocity
    Pose sum, prevPose = history.front();
    for (auto& pose : history) {
        sum.x += pose.x;
        sum.y += pose.y;
        sum.z += pose.z;
        sum.pitch += pose.pitch;
        sum.yaw += pose.yaw;
        sum.roll += pose.roll;

        // Update velocity using difference between consecutive poses
        velocity.x += (pose.x - prevPose.x);
        velocity.y += (pose.y - prevPose.y);
        velocity.z += (pose.z - prevPose.z);
        velocity.pitch += (pose.pitch - prevPose.pitch);
        velocity.yaw += (pose.yaw - prevPose.yaw);
        velocity.roll += (pose.roll - prevPose.roll);

        prevPose = pose;
    }

    // Average the position and scale the velocity
    int count = history.size();
    estimate.x = sum.x / count;
    estimate.y = sum.y / count;
    estimate.z = sum.z / count;
    estimate.pitch = sum.pitch / count;
    estimate.yaw = sum.yaw / count;
    estimate.roll = sum.roll / count;

    velocity.x /= (count - 1);
    velocity.y /= (count - 1);
    velocity.z /= (count - 1);
    velocity.pitch /= (count - 1);
    velocity.yaw /= (count - 1);
    velocity.roll /= (count - 1);
}

Pose KalmanFilter::getEstimate(){
    // Predict the next pose using the current estimate and velocity
    Pose p;
    p.x = estimate.x + velocity.x;
    p.y = estimate.y + velocity.y;
    p.z = estimate.z + velocity.z;
    p.pitch =  estimate.pitch + velocity.pitch;
    p.yaw = estimate.yaw + velocity.yaw;
    p.roll = estimate.roll + velocity.roll;
    return p;
}