/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PID.hpp                                                   */
/*    Author:       2775Josh, Modified by Connor White (WHOOP)                */
/*    Created:      Thu July 4 2024                                           */
/*    Description:  Whoop PID Calculator                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef PID_HPP
#define PID_HPP

/**
 * General-use PID class for drivetrains. It includes both
 * control calculation and settling calculation. The default
 * update period is 10ms or 100Hz
 */

class PID
{
private:
    double error = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double starti = 0;
    double settle_error = 0;
    double settle_time = 0;
    double timeout = 0;
    double accumulated_error = 0;
    double previous_error = 0;
    double output = 0;
    double time_spent_settled = 0;
    double time_spent_running = 0;
    double update_period = 10;

public:
    /**
     * PID constructor with settling inputs
     * The settling system works like this: The robot is settled
     * when error is less than settle_error for a duration of settle_time,
     * or if the function has gone on for longer than timeout. Otherwise
     * it is not settled. Starti keeps the I term at 0 until error is less
     * than starti
     *
     * @param error Difference in desired and current position
     * @param kp Proportional constant
     * @param ki Integral constant
     * @param kd Derivative constant
     * @param starti Maximum error to start integrating
     * @param settle_error Maximum error to be considered settled
     * @param settle_time Minimum time to be considered settled
     * @param timeout Time after which to give up and move on
     */
    PID(double error, double kp, double ki, double kd, double starti);

    /**
     * PID constructor with custom update period
     * The default update period is 10ms, but if you want to run
     * a faster or slower loop, you need to let the settler know
     *
     * @param error Difference in desired and current position
     * @param kp Proportional constant
     * @param ki Integral constant
     * @param kd Derivative constant
     * @param starti Maximum error to start integrating
     * @param settle_error Maximum error to be considered settled
     * @param settle_time Minimum time to be considered settled, in seconds
     * @param timeout Time after which to give up and move on, in seconds
     */
    PID(double error, double kp, double ki, double kd, double starti, double settle_error, double settle_time, double timeout);

    /**
     * Computes the output power based on the error
     * Typical PID calculation with some optimizations: When the robot crosses
     * error=0, the i-term gets reset to 0. And, of course, the robot only
     * accumulates i-term when error is less than starti. Read about these at
     * https://georgegillard.com/resources/documents
     *
     * @param error Difference in desired and current position
     * @return Output power
     */
    double step(double error);

    /**
     * Computes whether or not the movement has settled
     * The robot is considered settled when error is less than settle_error
     * for a duration of settle_time, or if the function has gone on for
     * longer than timeout. Otherwise it is not settled
     *
     * @return Whether the movement is settled
     */
    bool is_settled();
};

#endif // PID_HPP