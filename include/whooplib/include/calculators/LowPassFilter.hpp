/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       LowPassFilter.hpp                                         */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Low Pass Filter                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef LOW_PASS_FILTER_HPP
#define LOW_PASS_FILTER_HPP

class LowPassFilter
{
private:
    double alpha;     // Smoothing factor
    double value;     // Filtered value
    bool initialized; // Indicates if the filter has received at least one data point

public:
    /**
     * Constructor for lowpass filter
     * @param alpha The smoothing (tune-able) factor for the low-pass filter
     */
    LowPassFilter(double alpha);

    /**
     * Processes the data from low-pass filter. Meant to be ran recursively
     * @param newMeasurement The measurement to input into the filter
     * @return Processed filtered measurement
     */
    double process(double newMeasurement);
};

#endif // LOW_PASS_FILTER_HPP