/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       KalmanFilter.hpp                                          */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Kalman Filter Object                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/KalmanFilter.hpp"

KalmanFilter::KalmanFilter(double process_noise, double measurement_noise, double estimated_error, double initial_value)
    : Q(process_noise), R(measurement_noise), x(initial_value), P(estimated_error) {}

// Method to update the filter with a new measurement
double KalmanFilter::process(double newMeasurement)
{
    // Prediction update
    P = P + Q;

    // Measurement update
    K = P / (P + R);
    x = x + K * (newMeasurement - x);
    P = (1 - K) * P;

    return x;
}