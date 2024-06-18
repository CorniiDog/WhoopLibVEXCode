#include "vex.h"
#include "whooplib/include/devices/WhoopDrivetrain.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>


WhoopDrivetrain::WhoopDrivetrain(BufferNode* bufferSystem, std::string stream, WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors) : whoop_controller(controller){
    WhoopMotorGroup leftMotorGroup(leftMotors);
    left_motor_group = &leftMotorGroup;

    WhoopMotorGroup rightMotorGroup(rightMotors);
    right_motor_group = &rightMotorGroup;

    this->_assign_pose_stream(bufferSystem, stream);
} 


WhoopDrivetrain::WhoopDrivetrain(BufferNode* bufferSystem, std::string stream, WhoopController* controller,  WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup) : whoop_controller(controller), left_motor_group(leftMotorGroup), right_motor_group(rightMotorGroup){
    this->_assign_pose_stream(bufferSystem, stream);
}


void WhoopDrivetrain::set_state(drivetrainState state){
    drive_state = state;
}

void WhoopDrivetrain::_update_pose(std::string pose_data){
    std::istringstream iss(pose_data); // Create a string stream from the input string
    iss >> robotPose.x >> robotPose.y >> robotPose.z >> robotPose.pitch >> robotPose.yaw >> robotPose.roll;
}

void WhoopDrivetrain::_assign_pose_stream(BufferNode* bufferSystem, std::string stream){
    if(pose_messenger != nullptr){
        throw ("Welp... You can only assign one pose stream to the drivetrain.");
    }

    Messenger messenger(bufferSystem, stream, deleteAfterRead::no_delete);
    pose_messenger = &messenger;

    // Create a bound function for callback
    auto bound_function = std::bind(&WhoopDrivetrain::_update_pose, this, std::placeholders::_1);
    messenger.on_message(bound_function);
}

Pose WhoopDrivetrain::get_pose(){
    return robotPose;
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
    if(drive_state == drivetrainState::mode_disabled){
        left_motor_group->spin(0);
        right_motor_group->spin(0);
    }
}