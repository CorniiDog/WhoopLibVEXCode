#include "vex.h"
#include "whooplib/include/devices/WhoopInertial.hpp"
#include "whooplib/include/toolbox.hpp"

// Initialization Constructors
WhoopInertial::WhoopInertial(std::int32_t port) : vex_inertial(inertial(port)){} 

WhoopInertial::WhoopInertial(std::int32_t port, bool reversed) : vex_inertial(inertial(port)), is_reversed(reversed){} 

// Receiving rotation
double WhoopInertial::get_yaw(){
    if (is_reversed) 
        return -vex_inertial.heading(rotationUnits::deg)+yaw_offset;
    return vex_inertial.heading(rotationUnits::deg)+yaw_offset;
}

double WhoopInertial::get_yaw_degrees(){
    return this->get_yaw();
}

double WhoopInertial::get_yaw_radians(){
    return to_rad(this->get_yaw());
}

// Tare (reset)
void WhoopInertial::tare(){
    this->tare(0);
}
void WhoopInertial::tare(double degrees){
    yaw_offset = 0;
    vex_inertial.resetHeading();
}
void WhoopInertial::tare_degrees(double degrees){
    this->tare(degrees);
}
void WhoopInertial::tare_radians(double radians){
    this->tare(to_deg(radians));
}

