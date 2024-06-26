/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       RollingAverage.hpp                                        */
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
 * Rolling Average Filter Object
 */
class RollingAverageFilter {
private:
    std::deque<Pose> buffer;
    int capacity;

public:

    /**
   * Constructor for Rolling Average
   * @param n Number of elements for rolling average
   */
    RollingAverageFilter(int capacity);

    /**
   * Processes pose into rolling average
   * @return rolling average result
   */
    Pose process(const Pose& newMeasurement);
};

#endif