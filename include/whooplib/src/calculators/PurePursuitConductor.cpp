/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PurePursuitConductor.cpp                                  */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 3 2024                                           */
/*    Description:  Pure Pursuit Conductor for Motor Movements                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/PurePursuitConductor.hpp"

PurePursuitConductor::PurePursuitConductor(PursuitParams *default_pursuit_parameters) : turn_pid(0, default_pursuit_parameters->turning_kp, default_pursuit_parameters->turning_ki, default_pursuit_parameters->turning_kd, default_pursuit_parameters->turning_i_activation, default_pursuit_parameters->settle_rotation, default_pursuit_parameters->settle_time, default_pursuit_parameters->timeout),
                                                                                        forward_pid(0, default_pursuit_parameters->forward_kp, default_pursuit_parameters->forward_ki, default_pursuit_parameters->forward_kp, default_pursuit_parameters->forward_i_activation, default_pursuit_parameters->settle_distance, default_pursuit_parameters->settle_time, default_pursuit_parameters->timeout),
                                                                                        pursuit_path(TwoDPose(), TwoDPose(), default_pursuit_parameters->turning_radius, default_pursuit_parameters->lookahead_distance, default_pursuit_parameters->num_path_segments),
                                                                                        default_pursuit_parameters(default_pursuit_parameters)
{
}

void PurePursuitConductor::generate_path(TwoDPose start_position, TwoDPose destination_position)
{
    generate_path(start_position, destination_position, -1, -1);
}

void PurePursuitConductor::generate_path(TwoDPose start_position, TwoDPose destination_position, double timeout)
{
    generate_path(start_position, destination_position, timeout, -1);
}

void PurePursuitConductor::generate_path(TwoDPose start_position, TwoDPose destination_position, double timeout, double turning_radius)
{
    double turn_rad;
    if (turning_radius >= 0)
    {
        turn_rad = turning_radius;
    }
    else
    {
        turn_rad = default_pursuit_parameters->turning_radius;
    }

    double t_out;
    if (timeout >= 0)
    {
        t_out = timeout;
    }
    else
    {
        t_out = default_pursuit_parameters->timeout;
    }

    forward_pid = PID(0, default_pursuit_parameters->forward_kp, default_pursuit_parameters->forward_ki, default_pursuit_parameters->forward_kp, default_pursuit_parameters->forward_i_activation, default_pursuit_parameters->settle_distance, default_pursuit_parameters->settle_time, t_out),
    turn_pid = PID(0, default_pursuit_parameters->turning_kp, default_pursuit_parameters->turning_ki, default_pursuit_parameters->turning_kd, default_pursuit_parameters->turning_i_activation, default_pursuit_parameters->settle_rotation, default_pursuit_parameters->settle_time, t_out),

    pursuit_path = PurePursuitPath(start_position, destination_position, turn_rad, default_pursuit_parameters->lookahead_distance, default_pursuit_parameters->num_path_segments);

    enabled = true;
}

PursuitResult PurePursuitConductor::step(TwoDPose current_pose)
{
    if (!enabled)
    { // If not enabled, set is_valid as true and is_completed as true
        return PursuitResult(true, 0, 0, 0, 0, true);
    }

    PursuitEstimate estimate = pursuit_path.calculate_pursuit_estimate(current_pose);

    if (!estimate.is_valid)
    { // If error or something, return is_valid as false
        return PursuitResult(false, 0, 0, 0, 0, false);
    }

    double forward_power = forward_pid.step(estimate.distance);
    double turn_power = turn_pid.step(estimate.steering_angle);

    PursuitResult result = PursuitResult(true, estimate.steering_angle, estimate.distance, forward_power, turn_power, false);
    ;

    if (forward_pid.is_settled() && turn_pid.is_settled())
    {
        result.is_completed = true;
    }

    return result;
}