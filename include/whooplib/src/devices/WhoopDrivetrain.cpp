#include "vex.h"
#include "whooplib/include/devices/WhoopDrivetrain.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <memory> // For std::unique_ptr


WhoopDrivetrain::WhoopDrivetrain(Messenger* messenger, WhoopController* controller, WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup) 
: whoop_controller(controller), pose_messenger(messenger) {
    left_motor_group = std::make_unique<WhoopMotorGroup>(*leftMotorGroup);
    right_motor_group = std::make_unique<WhoopMotorGroup>(*rightMotorGroup);
}

WhoopDrivetrain::WhoopDrivetrain(Messenger* messenger, WhoopController* controller, std::vector<WhoopMotor*> left_motors, std::vector<WhoopMotor*> right_motors)
: whoop_controller(controller), pose_messenger(messenger) {
    left_motor_group = std::make_unique<WhoopMotorGroup>(left_motors);
    right_motor_group = std::make_unique<WhoopMotorGroup>(right_motors);
}

void WhoopDrivetrain::set_state(drivetrainState state){
    drive_state = state;
}

void WhoopDrivetrain::_update_pose(std::string pose_data){
    std::istringstream iss(pose_data); // Create a string stream from the input string
    iss >> pose.x >> pose.y >> pose.z >> pose.pitch >> pose.yaw >> pose.roll;
}


Pose WhoopDrivetrain::get_pose(){
    return pose;
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