#include "vex.h"
#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>

void WhoopMotorGroup::add_motor(WhoopMotor* motor){
    whoop_motors.push_back(motor);
}

WhoopMotorGroup::WhoopMotorGroup(std::vector<WhoopMotor*> motors){
    for (size_t i = 0; i < motors.size(); ++i) {
        add_motor(motors[i]);
    }
}

void WhoopMotorGroup::spin(double volts){
    for (size_t i = 0; i < whoop_motors.size(); ++i) {
        whoop_motors[i]->spin(volts);
    }
}

void WhoopMotorGroup::spin_percentage(double percentage){ // Percentage being -100 to 100
    for (size_t i = 0; i < whoop_motors.size(); ++i) {
        whoop_motors[i]->spin_percentage(percentage);
    }
}

void WhoopMotorGroup::spin_unit(double unit){ // Unit being -1 to 1, being 0 stopped
    for (size_t i = 0; i < whoop_motors.size(); ++i) {  
        whoop_motors[i]->spin_unit(unit);
    }
}

void WhoopMotorGroup::stop_hold(){
    for (size_t i = 0; i < whoop_motors.size(); ++i) {
        whoop_motors[i]->stop_hold();
    }
}

void WhoopMotorGroup::stop_brake(){
    for (size_t i = 0; i < whoop_motors.size(); ++i) {
        whoop_motors[i]->stop_brake();
    }
}

void WhoopMotorGroup::stop_coast(){
    for (size_t i = 0; i < whoop_motors.size(); ++i) {
        whoop_motors[i]->stop_coast();
    }
}

double WhoopMotorGroup::get_rotation(){
    double avg = 0;
    const int size = whoop_motors.size();

    for (size_t i = 0; i < size; ++i) {
        avg += whoop_motors[i]->get_rotation();
    }

    avg /= size;
    
    // If size is greater than 2, remove one outlier in case of a bad motor
    if (size > 2){

        // Find highest deviation motor to exclude
        int highest_dev_i = -1;
        double highest_dev = 0;
        double deviation;
        for (size_t i = 0; i < size; ++i) {
            deviation = std::abs(avg - whoop_motors[i]->get_rotation());
            if(deviation > highest_dev){
                highest_dev = deviation;
                highest_dev_i = i;
            }
        }

        // Apply, excluding one motor
        if(highest_dev_i != -1){
            avg = 0;
            for (size_t i = 0; i < size; ++i) {
                if(i != highest_dev_i){
                    avg += whoop_motors[i]->get_rotation();
                }
            }
            avg /= (size - 1);
        }
    }

    return avg;
}

double WhoopMotorGroup::get_rotation_degrees(){
    return this->get_rotation();
}

double WhoopMotorGroup::get_rotation_radians(){
    return to_rad(this->get_rotation());
}

void WhoopMotorGroup::tare(double degrees){
    for (size_t i = 0; i < whoop_motors.size(); ++i) {
        whoop_motors[i]->tare(degrees);
    }
}

void WhoopMotorGroup::tare(){
    this->tare(0);
}

void WhoopMotorGroup::tare_degrees(double degrees){
    this->tare(degrees);
}

void WhoopMotorGroup::tare_radians(double radians){
    this->tare(to_deg(radians));
}



