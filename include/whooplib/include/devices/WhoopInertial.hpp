#ifndef WHOOP_INERTIAL_HPP
#define WHOOP_INERTIAL_HPP

#include "vex.h"


// Declaration of BufferNode class
class WhoopInertial {
protected:
    // Upon initialization
    double yaw_offset = 0;
    bool is_reversed = false;
public:
    // Upon initialization
    inertial vex_inertial;

    // Initialization Constructors
    WhoopInertial(std::int32_t port); 
    WhoopInertial(std::int32_t port, bool reversed); 

    // Receiving rotation
    double get_yaw(); // Degrees is default
    double get_yaw_degrees();
    double get_yaw_radians();

    // Calibrate
    void calibrate();

    // Tare (reset)
    void tare();
    void tare(double degrees); // Degrees is default
    void tare_degrees(double degrees);
    void tare_radians(double radians);
};


#endif // WHOOP_MOTOR_HPP

