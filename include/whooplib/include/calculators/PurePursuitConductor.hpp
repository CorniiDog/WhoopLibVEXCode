/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PurePursuitConductor.hpp                                  */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 3 2024                                           */
/*    Description:  Pure Pursuit Conductor for Motor Movements                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef PURE_PURSUIT_CONDUCTOR_HPP
#define PURE_PURSUIT_CONDUCTOR_HPP

#include "whooplib/include/calculators/TwoDPose.hpp"
#include "whooplib/include/calculators/PurePursuit.hpp"
#include "whooplib/include/toolbox.hpp"

struct PursuitParams
{
    double turning_radius;
    double lookahead_distance;
    double destination_distance_threshold;
    double destination_rotation_threshold;

    double turning_kp;
    double turning_ki;
    double turning_kd;
    double turning_i_activation;

    double forward_kp;
    double forward_ki;
    double forward_kd;
    double forward_i_activation;

    double settle_time;
    double timeout;

    /**
     * @param turning_radius Radius of the turns, in meters
     * @param lookahead_distance Pure Pursuit look ahead distance
     * @param destination_distance_threshold Exits when within this distance of target, in meters
     * @param destination_rotation_threshold Exits when within this rotation of target, in radians
     * @param settle_time Minimum time to be considered settled, in seconds
     * @param timeout Time after which to give up and move on, in seconds
     * @param turning_kp Turning Proportional Tuning
     * @param turning_ki Turning Integral Tuning
     * @param turning_kd Turning Derivative Tuning
     * @param turning_i_activation The rotation distance (error), in radians, to activate turning_ki
     * @param forward_kp Forward Proportional Tuning
     * @param forward_ki Forward Integral Tuning
     * @param forward_kd Forward Derivative Tuning
     * @param forward_i_activation The rotation distance (error), in meters, to activate forward_ki
     */
    PursuitParams(double turning_radius = 0.2, double lookahead_distance = 0.1,
                  double destination_distance_threshold = 0.025, double destination_rotation_threshold = 0.09,
                  double settle_time = 2.0, double timeout = 5.0,
                  double turning_kp = 0.3, double turning_ki = 0.001, double turning_kd = 2, double turning_i_activation = to_meters(15),
                  double forward_kp = 1.5, double forward_ki = 0, double forward_kd = 10, double forward_i_activation = to_meters(0)) : turning_radius(turning_radius), lookahead_distance(lookahead_distance),
                                                                                                                                        destination_distance_threshold(destination_distance_threshold), destination_rotation_threshold(destination_rotation_threshold),
                                                                                                                                        settle_time(settle_time), timeout(timeout),
                                                                                                                                        turning_kp(turning_kp), turning_ki(turning_ki), turning_kd(turning_kd), turning_i_activation(turning_i_activation),
                                                                                                                                        forward_kp(forward_kp), forward_ki(forward_ki), forward_kd(forward_kd), forward_i_activation(forward_i_activation) {}
};




#endif