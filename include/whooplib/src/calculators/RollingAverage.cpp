/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       RollingAverage.cpp                                        */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Kalman Filter for Odometry                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/calculators/RollingAverage.hpp"

 RollingAverageFilter::RollingAverageFilter(int capacity) : capacity(capacity) {}

Pose RollingAverageFilter::process(const Pose& newMeasurement) {
    if (buffer.size() >= capacity) {
        buffer.pop_front();
    }
    buffer.push_back(newMeasurement);

    double sumX = 0, sumY = 0, sumZ = 0, sumPitch = 0, sumYaw = 0, sumRoll = 0, sumConfidence = 0;
    for (const auto& pose : buffer) {
        sumX += pose.x;
        sumY += pose.y;
        sumZ += pose.z;
        sumPitch += pose.pitch;
        sumYaw += pose.yaw;
        sumRoll += pose.yaw;
        sumConfidence += pose.confidence;
    }
    return Pose(sumX / buffer.size(), sumY / buffer.size(), sumZ / buffer.size(), sumPitch / buffer.size(), sumYaw / buffer.size(), sumRoll / buffer.size(), sumConfidence / buffer.size());
}