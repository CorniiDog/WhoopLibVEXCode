/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       RollingAverage.hpp                                        */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Rolling Average Filter for Odometry                       */
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
class RollingAverageFilter
{
private:
    std::deque<Pose> buffer_pose;
    std::deque<double> buffer_double;
    std::deque<double> buffer_int;
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
    Pose process(const Pose &newMeasurement);

    /**
     * Processes double into rolling average
     * @return rolling average result
     */
    double process(double newMeasurement);

    /**
     * Processes double into rolling average
     * @return rolling average result
     */
    int process(int newMeasurement);
};

#endif