/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDrivetrain.cpp                                       */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Drivetrain for Controlling Chassis                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/devices/WhoopDrivetrain.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <memory> // For std::unique_ptr

void WhoopDrivetrain::init_motor_groups(WhoopMotorGroup* leftGroup, WhoopMotorGroup* rightGroup) {
    left_motor_group = std::make_unique<WhoopMotorGroup>(*leftGroup);
    right_motor_group = std::make_unique<WhoopMotorGroup>(*rightGroup);
}

void WhoopDrivetrain::init_motor_groups(const std::vector<WhoopMotor*>& leftMotors, const std::vector<WhoopMotor*>& rightMotors) {
    left_motor_group = std::make_unique<WhoopMotorGroup>(leftMotors);
    right_motor_group = std::make_unique<WhoopMotorGroup>(rightMotors);
}

WhoopDrivetrain::WhoopDrivetrain(double wheel_diameter, WhoopController* controller, WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup)
: whoop_controller(controller) {
    init_motor_groups(leftMotorGroup, rightMotorGroup);
}

WhoopDrivetrain::WhoopDrivetrain(double wheel_diameter, WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors)
: whoop_controller(controller) {
    init_motor_groups(leftMotors, rightMotors);
}

WhoopDrivetrain::WhoopDrivetrain(double wheel_diameter, double gear_ratio, WhoopController* controller, WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup)
: WhoopDrivetrain(wheel_diameter, controller, leftMotorGroup, rightMotorGroup){
    set_gear_ratio_mult(gear_ratio);
}
WhoopDrivetrain::WhoopDrivetrain(double wheel_diameter, double gear_ratio, WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors)
: WhoopDrivetrain(wheel_diameter, controller, leftMotors, rightMotors){
    set_gear_ratio_mult(gear_ratio);
}

void WhoopDrivetrain::set_state(drivetrainState state){
    drive_state = state;
}

void WhoopDrivetrain::set_gear_ratio_mult(double ratio){ // motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0
    left_motor_group->set_gear_ratio_mult(ratio);
    right_motor_group->set_gear_ratio_mult(ratio);
}

void WhoopDrivetrain::__step(){

    // Controller input
    if(drive_state == drivetrainState::mode_usercontrol){
        if(whoop_controller->joystick_mode == joystickMode::joystickmode_tank){
            left_motor_group->spin_percentage(whoop_controller->get_left_joystick_y());
            right_motor_group->spin_percentage(whoop_controller->get_right_joystick_y());
        }
        else if(whoop_controller->joystick_mode == joystickMode::joystickmode_split_arcade){
            left_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() + whoop_controller->get_right_joystick_x());
            right_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() - whoop_controller->get_right_joystick_x());
        }
        else if(whoop_controller->joystick_mode == joystickMode::joystickmode_left_arcade){
            left_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() + whoop_controller->get_left_joystick_x());
            right_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() - whoop_controller->get_left_joystick_x());
        }
        else if(whoop_controller->joystick_mode == joystickMode::joystickmode_right_arcade){
            left_motor_group->spin_percentage(whoop_controller->get_right_joystick_y() + whoop_controller->get_right_joystick_x());
            right_motor_group->spin_percentage(whoop_controller->get_right_joystick_y() - whoop_controller->get_right_joystick_x());
        }
    }
    
    //Disabled
    else if(drive_state == drivetrainState::mode_disabled){
        left_motor_group->spin(0);
        right_motor_group->spin(0);
    }

}