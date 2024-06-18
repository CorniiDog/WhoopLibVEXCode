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

void WhoopDrivetrain::setup_messenger(BufferNode* bufferSystem, const std::string& pose_stream) {
    pose_messenger = std::make_unique<Messenger>(bufferSystem, pose_stream, deleteAfterRead::no_delete);
    pose_messenger->on_message(std::bind(&WhoopDrivetrain::_update_pose, this, std::placeholders::_1));
}

WhoopDrivetrain::WhoopDrivetrain(WhoopController* controller, WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup)
: whoop_controller(controller) {
    init_motor_groups(leftMotorGroup, rightMotorGroup);
}

WhoopDrivetrain::WhoopDrivetrain(WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors)
: whoop_controller(controller) {
    init_motor_groups(leftMotors, rightMotors);
}

WhoopDrivetrain::WhoopDrivetrain(BufferNode* bufferSystem, std::string pose_stream, WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors)
: WhoopDrivetrain(controller, leftMotors, rightMotors) {
    setup_messenger(bufferSystem, pose_stream);
}

WhoopDrivetrain::WhoopDrivetrain(BufferNode* bufferSystem, std::string pose_stream, WhoopController* controller,  WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup)
: WhoopDrivetrain(controller, leftMotorGroup, rightMotorGroup) {
    setup_messenger(bufferSystem, pose_stream);
}

WhoopDrivetrain::WhoopDrivetrain(double gear_ratio, WhoopController* controller, WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup)
: WhoopDrivetrain(controller, leftMotorGroup, rightMotorGroup){
    set_gear_ratio_mult(gear_ratio);
}
WhoopDrivetrain::WhoopDrivetrain(double gear_ratio, WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors)
: WhoopDrivetrain(controller, leftMotors, rightMotors){
    set_gear_ratio_mult(gear_ratio);
}
WhoopDrivetrain::WhoopDrivetrain(double gear_ratio, BufferNode* bufferSystem, std::string pose_stream, WhoopController* controller,  WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup)
: WhoopDrivetrain(bufferSystem, pose_stream, controller, leftMotorGroup, rightMotorGroup){
    set_gear_ratio_mult(gear_ratio);
}
WhoopDrivetrain::WhoopDrivetrain(double gear_ratio, BufferNode* bufferSystem, std::string pose_stream, WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors)
: WhoopDrivetrain(bufferSystem, pose_stream, controller, leftMotors, rightMotors){
    set_gear_ratio_mult(gear_ratio);
}

void WhoopDrivetrain::set_state(drivetrainState state){
    drive_state = state;
}

void WhoopDrivetrain::_update_pose(std::string pose_data){
    std::istringstream iss(pose_data); // Create a string stream from the input string
    iss >> pose.x >> pose.y >> pose.z >> pose.pitch >> pose.yaw >> pose.roll;
}

void WhoopDrivetrain::set_gear_ratio_mult(double ratio){ // motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0
    left_motor_group->set_gear_ratio_mult(ratio);
    right_motor_group->set_gear_ratio_mult(ratio);
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