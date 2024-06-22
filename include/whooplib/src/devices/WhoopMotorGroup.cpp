/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMotorGroup.cpp                                       */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Motor Group with Additional Reliability Features  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm> // Needed for std::max_element

void WhoopMotorGroup::add_motor(WhoopMotor* motor) {
    whoop_motors.push_back(motor);
}

WhoopMotorGroup::WhoopMotorGroup(std::vector<WhoopMotor*> motors) {
    for (auto& motor : motors) {
        add_motor(motor);
    }
}

void WhoopMotorGroup::apply_to_all(void (WhoopMotor::*func)(double), double value) {
    for (auto& motor : whoop_motors) {
        (motor->*func)(value);
    }
}

void WhoopMotorGroup::apply_to_all(void (WhoopMotor::*func)()) {
    for (auto& motor : whoop_motors) {
        (motor->*func)();
    }
}

void WhoopMotorGroup::spin(double volts) {
    apply_to_all(&WhoopMotor::spin, volts);
}

void WhoopMotorGroup::spin_percentage(double percentage) {
    apply_to_all(&WhoopMotor::spin_percentage, percentage);
}

void WhoopMotorGroup::spin_unit(double unit) {
    apply_to_all(&WhoopMotor::spin_unit, unit);
}

void WhoopMotorGroup::set_gear_ratio_mult(double ratio) {
    if (ratio <= 0) {
        throw std::invalid_argument("Gear ratio must be positive and non-zero.");
    }
    gear_ratio = ratio;
}

void WhoopMotorGroup::stop_hold() {
    apply_to_all(&WhoopMotor::stop_hold);
}

void WhoopMotorGroup::stop_brake() {
    apply_to_all(&WhoopMotor::stop_brake);
}

void WhoopMotorGroup::stop_coast() {
    apply_to_all(&WhoopMotor::stop_coast);
}

double WhoopMotorGroup::get_rotation() {
    if (whoop_motors.empty()) return 0;
    double total = 0;
    std::vector<double> rotations;
    for (auto& motor : whoop_motors) {
        double rotation = motor->get_rotation();
        rotations.push_back(rotation);
        total += rotation;
    }
    double avg = total / whoop_motors.size();
    
    // If there are more than 2 motors, remove a single outlier
    if (whoop_motors.size() > 2) {
        auto max_it = std::max_element(rotations.begin(), rotations.end(), [&avg](double a, double b) {
            return std::abs(a - avg) < std::abs(b - avg);
        });
        total -= *max_it;
        avg = total / (whoop_motors.size() - 1);
    }

    return avg * gear_ratio;
}

double WhoopMotorGroup::get_rotation_degrees() {
    return get_rotation();
}

double WhoopMotorGroup::get_rotation_radians() {
    return to_rad(get_rotation());
}

double WhoopMotorGroup::get_rotation_rotations(){
    return get_rotation_degrees()/360.0;
}

void WhoopMotorGroup::tare() {
    apply_to_all(&WhoopMotor::tare, 0);
}

void WhoopMotorGroup::tare(double degrees) {
    apply_to_all(&WhoopMotor::tare, degrees / gear_ratio);
}

void WhoopMotorGroup::tare_degrees(double degrees) {
    tare(degrees);
}

void WhoopMotorGroup::tare_rotations(double rotations) {
    tare(rotations*360.0);
}

void WhoopMotorGroup::tare_radians(double radians) {
    tare(to_deg(radians));
}


