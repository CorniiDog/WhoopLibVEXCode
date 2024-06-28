/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopController.cpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Controller With Additional Functions              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/toolbox.hpp"
#include "whooplib/include/devices/WhoopController.hpp"
#include <vector>
#include <functional>
#include <cmath>

// Initialization Constructors
WhoopController::WhoopController(joystickMode mode) : vex_controller(controller(controllerType::primary)), joystick_mode(mode){} 

WhoopController::WhoopController(joystickMode mode, vex::controllerType controller_type) : vex_controller(controller(controller_type)), joystick_mode(mode){} 


void WhoopController::notify(std::string message, double duration_seconds){
    vex_controller.Screen.clearLine(1);
    vex_controller.Screen.setCursor(1, 1);
    vex_controller.Screen.print("%s", message.c_str());
    vex_controller.rumble(".");

    time_left_to_clear = duration_seconds * std::round(1000 / step_time_ms);
}

/////////////////////////////////////////////
// Controller joystick
double WhoopController::get_left_joystick_x(){
    return vex_controller.Axis4.position(pct);
}
double WhoopController::get_left_joystick_y(){
    return vex_controller.Axis3.position(pct);
}
double WhoopController::get_right_joystick_x(){
    return vex_controller.Axis1.position(pct);
}
double WhoopController::get_right_joystick_y(){
    return vex_controller.Axis2.position(pct);
}

/////////////////////////////////////////////
//UDLR Reading
bool WhoopController::up_pressing(){
    return vex_controller.ButtonUp.pressing();
}
bool WhoopController::down_pressing(){
    return vex_controller.ButtonDown.pressing();
}
bool WhoopController::left_pressing(){
    return vex_controller.ButtonLeft.pressing();
}
bool WhoopController::right_pressing(){
    return vex_controller.ButtonRight.pressing();
}

/////////////////////////////////////////////
//ABXY Reading
bool WhoopController::a_pressing(){
    return vex_controller.ButtonA.pressing();
}
bool WhoopController::b_pressing(){
    return vex_controller.ButtonB.pressing();
}
bool WhoopController::x_pressing(){
    return vex_controller.ButtonX.pressing();
}
bool WhoopController::y_pressing(){
    return vex_controller.ButtonY.pressing();
}

/////////////////////////////////////////////
//Bumpers Reading
bool WhoopController::right_top_bumper_pressing(){
    return vex_controller.ButtonR1.pressing();
}
bool WhoopController::right_bottom_bumper_pressing(){
    return vex_controller.ButtonR2.pressing();
}
bool WhoopController::left_top_bumper_pressing(){
    return vex_controller.ButtonL1.pressing();
}
bool WhoopController::left_bottom_bumper_pressing(){
    return vex_controller.ButtonL2.pressing();
}

/////////////////////////////////////////////
//UDLR Events

//UDLR Pressed Events
void WhoopController::on_up_pressed_event(void (*callback)()){
    vex_controller.ButtonUp.pressed(callback);
}
void WhoopController::on_down_pressed_event(void (*callback)()){
    vex_controller.ButtonDown.pressed(callback);
}
void WhoopController::on_left_pressed_event(void (*callback)()){
    vex_controller.ButtonLeft.pressed(callback);
}
void WhoopController::on_right_pressed_event(void (*callback)()){
    vex_controller.ButtonRight.pressed(callback);
}

//UDLR Released Events
void WhoopController::on_up_released_event(void (*callback)()){
    vex_controller.ButtonUp.released(callback);
}
void WhoopController::on_down_released_event(void (*callback)()){
    vex_controller.ButtonDown.released(callback);
}
void WhoopController::on_left_released_event(void (*callback)()){
    vex_controller.ButtonLeft.released(callback);
}
void WhoopController::on_right_released_event(void (*callback)()){
    vex_controller.ButtonRight.released(callback);
}

/////////////////////////////////////////////
//ABXY Events

//ABXY Pressed Events
void WhoopController::on_a_pressed_event(void (*callback)()){
    vex_controller.ButtonA.pressed(callback);
}
void WhoopController::on_b_pressed_event(void (*callback)()){
    vex_controller.ButtonB.pressed(callback);
}
void WhoopController::on_x_pressed_event(void (*callback)()){
    vex_controller.ButtonX.pressed(callback);
}
void WhoopController::on_y_pressed_event(void (*callback)()){
    vex_controller.ButtonY.pressed(callback);
}

//ABXY Released Events
void WhoopController::on_a_released_event(void (*callback)()){
    vex_controller.ButtonA.released(callback);
}
void WhoopController::on_b_released_event(void (*callback)()){
    vex_controller.ButtonB.released(callback);
}
void WhoopController::on_x_released_event(void (*callback)()){
    vex_controller.ButtonX.released(callback);
}
void WhoopController::on_y_released_event(void (*callback)()){
    vex_controller.ButtonY.released(callback);
}

/////////////////////////////////////////////
//Bumper Events

//Bumper Pressed Events
void WhoopController::on_right_top_bumper_pressed_event(void (*callback)()){
    vex_controller.ButtonR1.pressed(callback);
}
void WhoopController::on_right_bottom_bumper_pressed_event(void (*callback)()){
    vex_controller.ButtonR2.pressed(callback);
}
void WhoopController::on_left_top_bumper_pressed_event(void (*callback)()){
    vex_controller.ButtonL1.pressed(callback);
}
void WhoopController::on_left_bottom_bumper_pressed_event(void (*callback)()){
    vex_controller.ButtonL2.pressed(callback);
}

//Bumper Released Events
void WhoopController::on_right_top_bumper_released_event(void (*callback)()){
    vex_controller.ButtonR1.released(callback);
}
void WhoopController::on_right_bottom_bumper_released_event(void (*callback)()){
    vex_controller.ButtonR2.released(callback);
}
void WhoopController::on_left_top_bumper_released_event(void (*callback)()){
    vex_controller.ButtonL1.released(callback);
}
void WhoopController::on_left_bottom_bumper_released_event(void (*callback)()){
    vex_controller.ButtonL2.released(callback);
}


void WhoopController::__step(){
    time_left_to_clear -= 1;

    if(time_left_to_clear < -1){
        time_left_to_clear = -1;
    }

    if(time_left_to_clear == 0){
        vex_controller.Screen.clearLine(1);
    }
}