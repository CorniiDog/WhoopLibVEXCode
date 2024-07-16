/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMotor.cpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Motor With Streamlined Functions                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/toolbox.hpp"

WhoopMotor::WhoopMotor(std::int32_t port) : vex_motor(vex::motor(port)) {}

WhoopMotor::WhoopMotor(std::int32_t port, reversed reversed) : vex_motor(vex::motor(port, vex::ratio18_1, reversed)) {}

WhoopMotor::WhoopMotor(std::int32_t port, vex::gearSetting gearRatio) : vex_motor(vex::motor(port, gearRatio)) {}

WhoopMotor::WhoopMotor(std::int32_t port, vex::gearSetting gearRatio, reversed reversed) : vex_motor(vex::motor(port, gearRatio, reversed)) {}

void WhoopMotor::spin(double volts)
{
    // Linearizes the voltage. Visual representation of the linearization: https://www.desmos.com/calculator/anyejul5wg
    // It attempts to make the voltage and motor power more linearly proportional
    vex_motor.spin(fwd, linearize_voltage(volts), voltageUnits::volt);
}

void WhoopMotor::spin_unit(double unit)
{ // Unit being -1 to 1, being 0 stopped
    vex_motor.spin(fwd, unit * 12.0, voltageUnits::volt);
}

void WhoopMotor::spin_percentage(double percentage)
{ // Percentage being -100 to 100
    this->spin_unit(percentage / 100.0);
}

void WhoopMotor::stop_hold()
{
    vex_motor.stop(brakeType::hold);
}

void WhoopMotor::stop_brake()
{
    vex_motor.stop(brakeType::brake);
}

void WhoopMotor::stop_coast()
{
    vex_motor.stop(brakeType::coast);
}

double WhoopMotor::get_rotation()
{
    return vex_motor.position(rotationUnits::deg) + pos_offset;
}

double WhoopMotor::get_rotation_rotations()
{
    return this->get_rotation() / 360.0;
}

double WhoopMotor::get_rotation_degrees()
{
    return this->get_rotation();
}

double WhoopMotor::get_rotation_radians()
{
    return to_rad(this->get_rotation());
}

// Receiving velocity
double WhoopMotor::get_velocity(vex::velocityUnits vel)
{
    return vex_motor.velocity(vel);
}

double WhoopMotor::get_velocity_deg_s()
{
    return get_velocity();
}

double WhoopMotor::get_velocity_rad_s()
{
    return to_rad(get_velocity());
}

double WhoopMotor::get_velocity_rpm()
{
    return vex_motor.velocity(velocityUnits::rpm);
}

void WhoopMotor::tare(double degrees)
{
    pos_offset = degrees;
    vex_motor.resetPosition();
}

void WhoopMotor::tare()
{
    this->tare(0);
}

void WhoopMotor::tare_degrees(double degrees)
{
    this->tare(degrees);
}

void WhoopMotor::tare_rotations(double rotations)
{
    this->tare(rotations * 360.0);
}

void WhoopMotor::tare_radians(double radians)
{
    this->tare(to_deg(radians));
}
