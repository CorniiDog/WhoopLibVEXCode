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
    float error = 0;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float starti = 0;
    float settle_error = 0;
    float settle_time = 0;
    float timeout = 0;
    float accumulated_error = 0;
    float previous_error = 0;
    float output = 0;
    float time_spent_settled = 0;
    float time_spent_running = 0;
    float update_period = 10;

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
    PID(float error, float kp, float ki, float kd, float starti);

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
    PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout);

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
    float step(float error);

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