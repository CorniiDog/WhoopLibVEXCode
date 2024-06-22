/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMotorGroup.hpp                                       */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Motor Group with Additional Reliability Features  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_MOTOR_GROUP_HPP
#define WHOOP_MOTOR_GROUP_HPP

#include "whooplib/include/devices/WhoopMotor.hpp"

#include "vex.h"
#include <vector>


/**
 * Manages a group of WhoopMotors, allowing synchronized control over multiple motors.
 */
class WhoopMotorGroup {
private:
    /**
     * Applies a specified member function to all motors in the group with a double value argument.
     * @param func Pointer to the member function to be applied.
     * @param value The double value to pass to the function.
     */
    void apply_to_all(void (WhoopMotor::*func)(double), double value);

    /**
     * Applies a specified member function to all motors in the group without arguments.
     * @param func Pointer to the member function to be applied.
     */
    void apply_to_all(void (WhoopMotor::*func)());
protected:
    // Upon initialization
    std::vector<WhoopMotor*> whoop_motors; // Vector of pointers to WhoopMotor objects.

    double gear_ratio = 1; // Gear ratio for scaling motor output, default is 1.
public:

    /**
     * Constructor that initializes a motor group with a vector of motors.
     * @param whoop_motors Vector of pointers to initialized WhoopMotors.
     */
    WhoopMotorGroup(std::vector<WhoopMotor*> whoop_motors);

    /**
     * Adds a motor to the motor group.
     * @param whoop_motor Pointer to the WhoopMotor to be added to the group.
     */
    void add_motor(WhoopMotor* whoop_motor);

    // Motor commands
    void spin(double volts); // Commands all motors to spin at a specified voltage. -12.0 -> 0 -> 12.0, with 0 being stopped.
    void spin_unit(double unit); // Commands all motors to spin at a unit -1 -> 0 -> 1, with 0 being stopped.
    void spin_percentage(double percentage); // Commands all motors to spin at a specified percentage -100 -> 0 -> 100, with 0 being stopped.
    void stop_hold(); // Commands all motors to stop and hold their position.
    void stop_brake(); // Commands all motors to stop with braking.
    void stop_coast(); // Commands all motors to stop and coast.

    /**
     * Sets the gear ratio multiplier for the motor group.
     * i.e. motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0 = 0.5
     * @param ratio The gear ratio multiplier to set.
     */
    void set_gear_ratio_mult(double ratio); // motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0

    // Receiving rotation
    // Note: If 3 or more motors are in a motor group, the motor group would get the average of n-1 motors (excluding the outlier motor furthest from average).
    double get_rotation(); // Returns the average rotation across all motors in degrees.
    double get_rotation_degrees(); // Returns the average rotation across all motors in degrees.
    double get_rotation_radians(); // Returns the average rotation across all motors in radians.
    double get_rotation_rotations(); // Returns the average rotation across all motors in full rotations.

    // Tare (reset)
    void tare(); // Resets the encoder count for all motors in the group.
    void tare(double degrees); // Resets the encoder count for all motors to a specified degree value.
    void tare_degrees(double degrees); // Resets the encoder count for all motors to a specified degree value.
    void tare_radians(double radians); // Resets the encoder count for all motors to a specified radian value.
    void tare_rotations(double rotations); // Resets the encoder count for all motors to a specified number of full rotations.
};


#endif // WHOOP_MOTOR_GROUP_HPP

