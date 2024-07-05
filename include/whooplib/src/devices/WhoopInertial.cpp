/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopInertial.cpp                                         */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Interial Sensor that Follow Robotic Standard      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/devices/WhoopInertial.hpp"
#include "whooplib/include/toolbox.hpp"

#include "whooplib/include/devices/WhoopMotor.hpp"

// Initialization Constructors
WhoopInertial::WhoopInertial(std::int32_t port) : vex_inertial(inertial(port)){} 

WhoopInertial::WhoopInertial(std::int32_t port, double correction_multiplier): 
    vex_inertial(inertial(port))
{
    this->correction_multiplier = correction_multiplier;
}

// Receiving rotation
double WhoopInertial::get_yaw(){
    double yaw = -(vex_inertial.heading(rotationUnits::deg) * correction_multiplier);

    yaw += yaw_offset;

    // Normalize the yaw to be within -180 to 180 degrees
    yaw = fmod(yaw + 180, 360);
    if (yaw < 0) {
        yaw += 360;
    }
    yaw -= 180;

    return yaw;
}

double WhoopInertial::get_yaw_degrees(){
    return this->get_yaw();
}

double WhoopInertial::get_yaw_radians(){
    return to_rad(this->get_yaw());
}


double WhoopInertial::get_roll(){
    return vex_inertial.roll();
}

double WhoopInertial::get_roll_degrees(){
    return get_roll();
}

double WhoopInertial::get_roll_radians(){
    return to_rad(get_roll());
}

double WhoopInertial::get_pitch(){
    return vex_inertial.pitch();
}

double WhoopInertial::get_pitch_degrees(){
    return get_pitch();
}

double WhoopInertial::get_pitch_radians(){
    return to_rad(get_pitch());
}

// Calibrate
void WhoopInertial::calibrate(){
    vex_inertial.calibrate();
}

// Tare (reset)
void WhoopInertial::tare(){
    this->tare(0);
}
void WhoopInertial::tare(double degrees){
    yaw_offset = degrees;
    vex_inertial.resetHeading();
}
void WhoopInertial::tare_degrees(double degrees){
    this->tare(degrees);
}
void WhoopInertial::tare_radians(double radians){
    this->tare(to_deg(radians));
}

