#ifndef WHOOP_MOTOR_HPP
#define WHOOP_MOTOR_HPP

#include "vex.h"

enum reversed{
    no_reverse=true,
    yes_reverse=false
};

// Declaration of WhoopMotor class
class WhoopMotor {
protected:
    // Upon initialization
    double pos_offset = 0;
public:
    // Upon initialization
    motor vex_motor;

    // Initialization Constructors
    WhoopMotor(std::int32_t port); 
    WhoopMotor(std::int32_t port, reversed reversed); 
    WhoopMotor(std::int32_t port, vex::gearSetting gearRatio);
    WhoopMotor(std::int32_t port, vex::gearSetting gearRatio, reversed reversed);

    // Motor commands
    void spin(double volts);
    void spin_unit(double unit); // Unit being -1 to 1, being 0 stopped
    void spin_percentage(double percentage); // -100 to 100
    void stop_hold();
    void stop_brake();
    void stop_coast();

    // Receiving rotation
    double get_rotation(); // Degrees is default
    double get_rotation_degrees();
    double get_rotation_radians();
    double get_rotation_rotations();

    // Tare (reset)
    void tare();
    void tare(double degrees); // Degrees is default
    void tare_degrees(double degrees);
    void tare_radians(double radians);
    void tare_rotations(double rotations);
};


#endif // WHOOP_MOTOR_HPP

