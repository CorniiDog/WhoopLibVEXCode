/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMotor.hpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Motor With Streamlined Functions                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_MOTOR_HPP
#define WHOOP_MOTOR_HPP

#include "vex.h"

/**
 * Enum to specify whether the motor should be reversed.
 */
enum reversed
{
    no_reverse = true,
    yes_reverse = false
};

/**
 * Represents a motor with control over its speed, direction, and measurement capabilities.
 */
class WhoopMotor
{
protected:
    double pos_offset = 0; // Offset applied to the position readings of the motor.
public:
    // Upon initialization
    motor vex_motor; // VEX motor instance.

    /**
     * Constructor to initialize a motor on a specified port.
     * @param port The port number where the motor is connected.
     */
    WhoopMotor(std::int32_t port);

    /**
     * Constructor to initialize a motor with an option to reverse its direction.
     * @param port The port number where the motor is connected.
     * @param reversed Enum value to set the motor direction reversed or not.
     */
    WhoopMotor(std::int32_t port, reversed reversed);

    /**
     * Constructor to initialize a motor with a specified gear ratio.
     * @param port The port number where the motor is connected.
     * @param gearRatio The gear setting of the motor.
     */
    WhoopMotor(std::int32_t port, vex::gearSetting gearRatio);

    /**
     * Constructor to initialize a motor with a gear ratio and an option to reverse its direction.
     * @param port The port number where the motor is connected.
     * @param gearRatio The gear setting of the motor.
     * @param reversed Enum value to set the motor direction reversed or not.
     */
    WhoopMotor(std::int32_t port, vex::gearSetting gearRatio, reversed reversed);

    // Motor commands
    void spin(double volts);                 // Commands the motor to spin at a voltage (-12.0 to 12.0, with 0.0 being stopped).
    void spin_unit(double unit);             // Commands the motor to spin in a unit range from -1.0 to 1.0, with being 0.0 stopped.
    void spin_percentage(double percentage); // Commands the motor to spin at a percentage of its capacity from -100.0 to 100.0, with 0.0 being stopped..
    void stop_hold();                        // Stops the motor and holds its position.
    void stop_brake();                       // Stops the motor with braking.
    void stop_coast();                       // Stops the motor and allows it to coast.

    // Receiving rotation
    double get_rotation();           // Degrees is default
    double get_rotation_degrees();   // Returns the current motor rotation in degrees.
    double get_rotation_radians();   // Returns the current motor rotation in radians.
    double get_rotation_rotations(); // Returns the current motor rotation in full rotations.

    // Receiving velocity
    double get_velocity(vex::velocityUnits vel = vex::velocityUnits::dps); // degrees/sec is default
    double get_velocity_deg_s();                                           // explicitly defining degrees/sec
    double get_velocity_rad_s();                                           // explicitly defining rad/sec
    double get_velocity_rpm();                                             // explicitly defining rpm

    // Tare (reset)
    void tare();
    void tare(double degrees);             // Degrees is default
    void tare_degrees(double degrees);     // Resets the motor encoder count to a specified degree.
    void tare_radians(double radians);     // Resets the motor encoder count to a specified radian.
    void tare_rotations(double rotations); // Resets the motor encoder count to a specified number of rotations.
};

#endif // WHOOP_MOTOR_HPP
