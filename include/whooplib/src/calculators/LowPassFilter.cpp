/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       LowPassFilter.cpp                                         */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Low Pass Filter                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/LowPassFilter.hpp"


LowPassFilter::LowPassFilter(double alpha): alpha(alpha), value(0.0), initialized(false) {}

double LowPassFilter::process(double newMeasurement) {
    if (!initialized) {
        // Initialize with the first data point
        value = newMeasurement;
        initialized = true;
    } else {
        // Apply the low pass filter formula
        value = alpha * newMeasurement + (1 - alpha) * value;
    }
    return value;
}