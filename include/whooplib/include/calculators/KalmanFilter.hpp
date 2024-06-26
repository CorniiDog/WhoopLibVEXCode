/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       KalmanFilter.hpp                                          */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Kalman Filter for Odometry                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "vex.h"
#include "whooplib/include/devices/WhoopVision.hpp"
#include <deque>

/**
 * Kalman Filter Object
 */
class KalmanFilter {
private:
    std::deque<Pose> history;
    int capacity;
    Pose estimate;
    Pose velocity;

public:
    /**
   * Kalman Filter Constructor
   * @param n The number of elements to filter (If we are doing 100Hz system with n=5 elements, delay would be 0.05 seconds, or be equivalent of 20Hz system)
   */
    KalmanFilter(int n);

    /**
   * Adds measurement to the Kalman filter
   * @param newPose The pose to record for the filter
   */
    void addMeasurement(const Pose& newPose);

    /**
   * Returns the estimated pose, including a forecast.
   * @return The smoothened out + forwarded pose
   */
    Pose getEstimate();
};

#endif