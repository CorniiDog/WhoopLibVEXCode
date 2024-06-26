/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       KalmanFilter.hpp                                          */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Kalman Filter Object                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

class KalmanFilter {
private:
    double Q;  // Process noise covariance
    double R;  // Measurement noise covariance
    double x;  // Estimated state
    double P;  // Estimation error covariance
    double K;  // Kalman Gain

public:
    /**
     * Constructor for Kalman Filter
     * @param process_noise Process noise covariance
     * @param measurement_noise Measurement noise covariance
     * @param estimated_error Estimation error covariance
     * @param initial_value The initial value of the filter
     */
    KalmanFilter(double process_noise, double measurement_noise, double estimated_error, double initial_value);

    /**
     * Processes the data from Kalman filter. Meant to be ran recursively
     * @param newMeasurement The measurement to input into the filter
     * @return Processed filtered measurement
     */
    double process(double newMeasurement);
};

#endif // KALMAN_FILTER_HPP