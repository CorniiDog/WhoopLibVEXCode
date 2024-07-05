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
#include "whooplib/include/calculators/PID.hpp"
#include "whooplib/include/toolbox.hpp"
#include <vector>

struct PursuitParams
{
    double turning_radius;
    double lookahead_distance;

    double settle_distance;
    double settle_rotation;
    double settle_time;
    double timeout;

    double turning_kp;
    double turning_ki;
    double turning_kd;
    double turning_i_activation;

    double forward_kp;
    double forward_ki;
    double forward_kd;
    double forward_i_activation;

    int num_path_segments;

    /**
     * @param turning_radius Radius of the turns, in meters
     * @param lookahead_distance Pure Pursuit look ahead distance, in meters
     * @param settle_distance Exits when within this distance of target, in meters
     * @param settle_rotation Exits when within this rotation of target, in radians
     * @param settle_time Minimum time to be considered settled, in seconds
     * @param timeout Time after which to give up and move on, in seconds
     * @param turning_kp Turning Proportional Tuning
     * @param turning_ki Turning Integral Tuning
     * @param turning_kd Turning Derivative Tuning
     * @param turning_i_activation The rotation distance (error), in radians, to activate turning_ki
     * @param forward_kp Forward Proportional Tuning
     * @param forward_ki Forward Integral Tuning
     * @param forward_kd Forward Derivative Tuning
     * @param forward_i_activation The forward distance (error), in meters, to activate forward_ki
     * @param num_path_segments The number of points when generating the path. More points mean higher detail of the path, but at a higher computational cost
     */
    PursuitParams(double turning_radius = 0.2, double lookahead_distance = 0.1,
                  double settle_distance = 0.025, double settle_rotation = 0.09,
                  double settle_time = 0.4, double timeout = 0,
                  double turning_kp = 0.3, double turning_ki = 0.001, double turning_kd = 2, double turning_i_activation = to_meters(15),
                  double forward_kp = 1.5, double forward_ki = 0, double forward_kd = 10, double forward_i_activation = to_meters(0),
                  int num_path_segments = 200) : turning_radius(turning_radius), lookahead_distance(lookahead_distance),
                                                 settle_distance(settle_distance), settle_rotation(settle_rotation),
                                                 settle_time(settle_time), timeout(timeout),
                                                 turning_kp(turning_kp), turning_ki(turning_ki), turning_kd(turning_kd), turning_i_activation(turning_i_activation),
                                                 forward_kp(forward_kp), forward_ki(forward_ki), forward_kd(forward_kd), forward_i_activation(forward_i_activation),
                                                 num_path_segments(num_path_segments) {}
};

struct PursuitResult
{
    bool is_valid;
    double steering_angle;
    double distance;
    double forward_power;
    double steering_power;
    bool is_completed;
    /**
     * @param is_valid would be true if the pursuit estimate returned no error
     * @param steering_angle would be the angle to turn towards for course correction in radians, counter-clockwise-positive
     * @param distance would be the distance from the target, in meters
     * @param forward_power the suggested motor power to go forward
     * @param steering_power the suggestedmotor power for steering
     * @param is_completed if true, the pure pursuit is complete
     */
    PursuitResult(bool is_valid = false, double steering_angle = 0, double distance = 0, double forward_power = 0, double steering_power = 0, bool is_completed = false) : is_valid(is_valid), steering_angle(steering_angle), distance(distance), forward_power(forward_power), steering_power(steering_power), is_completed(is_completed) {}
};

class PurePursuitConductor
{
private:
    PID turn_pid;
    PID forward_pid;
public:
    PurePursuitPath pursuit_path;
private:
    PursuitParams *default_pursuit_parameters = nullptr;
public:
    bool enabled = false;

    /**
     * Constructs the conductor for the pure pursuit object
     * @param default_pursuit_parameters The parameters for the pure pursuit
     */
    PurePursuitConductor(PursuitParams *default_pursuit_parameters);

    /**
     * Generates the path for the point
     * @param start_position The TwoDPose of the start position
     * @param destination_position The TwoDPose of the destination position
     */
    void generate_path(TwoDPose start_position, TwoDPose destination_position);

    /**
     * Generates the path for the point
     * @param start_position The TwoDPose of the start position
     * @param destination_position The TwoDPose of the destination position
     * @param timeout The timeout of the movement, in seconds
     */
    void generate_path(TwoDPose start_position, TwoDPose destination_position, double timeout);

    /**
     * Generates the path for the point
     * @param start_position The TwoDPose of the start position
     * @param destination_position The TwoDPose of the destination position
     * @param timeout The timeout of the movement, in seconds
     * @param turning_radius The radius, in meters, of the turning
     */
    void generate_path(TwoDPose start_position, TwoDPose destination_position, double timeout, double turning_radius);

    /**
     * Steps the conductor
     */
    PursuitResult step(TwoDPose current_pose);
};

#endif