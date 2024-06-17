#ifndef WHOOP_MOTOR_HPP
#define WHOOP_MOTOR_HPP

#include "vex.h"


// Declaration of BufferNode class
class WhoopMotor {
protected:
    // Upon initialization
    double pos_offset = 0;
public:
    // Upon initialization
    motor vex_motor;

    // Initialization Constructors
    WhoopMotor(std::int32_t port); 
    WhoopMotor(std::int32_t port, bool reversed); 
    WhoopMotor(std::int32_t port, vex::gearSetting gearRatio);
    WhoopMotor(std::int32_t port, vex::gearSetting gearRatio, bool reversed);

    // Motor commands
    void spin(double volts);
    void stop_hold();
    void stop_brake();
    void stop_coast();

    // Receiving rotation
    double get_rotation(); // Degrees is default
    double get_rotation_degrees();
    double get_rotation_radians();

    // Tare (reset)
    void tare();
    void tare(double degrees); // Degrees is default
    void tare_degrees(double degrees);
    void tare_radians(double radians);
};


#endif // WHOOP_MOTOR_HPP

