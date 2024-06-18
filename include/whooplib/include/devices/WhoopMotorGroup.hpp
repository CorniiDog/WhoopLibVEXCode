#ifndef WHOOP_MOTOR_GROUP_HPP
#define WHOOP_MOTOR_GROUP_HPP

#include "whooplib/include/devices/WhoopMotor.hpp"

#include "vex.h"
#include <vector>


// Declaration of WhoopMotorGroup class
class WhoopMotorGroup {
private:
    void apply_to_all(void (WhoopMotor::*func)(double), double value);
    void apply_to_all(void (WhoopMotor::*func)());
protected:
    // Upon initialization
    std::vector<WhoopMotor*> whoop_motors;

    double gear_ratio = 1;
public:

    void add_motor(WhoopMotor* whoop_motor);

    // Initialization Constructors
    WhoopMotorGroup(std::vector<WhoopMotor*> whoop_motors);

    // Motor commands
    void spin(double volts);
    void spin_unit(double unit);
    void spin_percentage(double percentage);
    void stop_hold();
    void stop_brake();
    void stop_coast();

    void set_gear_ratio_mult(double ratio); // motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0

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

