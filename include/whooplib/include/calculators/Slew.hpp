/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Slew.hpp                                                  */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 4 2024                                           */
/*    Description:  Slew Calculator                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef SLEW_HPP
#define SLEW_HPP

/**
 * General-use PID class for drivetrains. It includes both
 * control calculation and settling calculation. The default
 * update period is 10ms or 100Hz
 */

class Slew
{
public:
    double slewed_error = 0;
    double max_increase_per_second;
    double max_increase_per_step;
    bool can_slew_down;
    int milliseconds_per_step;

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
     * @param can_slew_down Set to false to disable slewing if slowing down. But set to true to slew downwards too
     * @param milliseconds_per_step The amount of milliseconds per step (recommended to be 10ms)
     */
    Slew(double max_increase_per_second, bool can_slew_down = false, int milliseconds_per_step=10);

    /**
     * Steps and computes
     * @param error Difference in desired and current position
     * @return Output power
     */
    double step(double error);
};

#endif // SLEW_HPP