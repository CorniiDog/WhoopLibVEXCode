#ifndef WHOOP_CONTROLLER_HPP
#define WHOOP_CONTROLLER_HPP

#include "vex.h"
#include <vector>
#include <functional>

enum joystickMode{
    joystickmode_tank = 1,
    joystickmode_split_arcade = 2,
    joystickmode_left_arcade = 3,
    joystickmode_right_arcade = 4
}; 


// Declaration of WhoopController class
class WhoopController {
protected:
    vex::controller vex_controller;
    joystickMode joystick_mode;
public:

    // Initialization Constructors
    WhoopController(joystickMode mode); 
    WhoopController(joystickMode mode, vex::controllerType controller_type); 

    // Controller joystick
    double get_left_joystick_x();
    double get_left_joystick_y();
    double get_right_joystick_x();
    double get_right_joystick_y();

    //UDLR
    bool up_pressing();
    bool down_pressing();
    bool left_pressing();
    bool right_pressing();

    //ABXY
    bool a_pressing();
    bool b_pressing();
    bool x_pressing();
    bool y_pressing();

    //Bumpers
    bool right_top_bumper_pressing();
    bool right_bottom_bumper_pressing();
    bool left_top_bumper_pressing();
    bool left_bottom_bumper_pressing();

    //UDLR Events
    void on_up_pressed_event(void (*callback)());
    void on_down_pressed_event(void (*callback)());
    void on_left_pressed_event(void (*callback)());
    void on_right_pressed_event(void (*callback)());

    void on_up_released_event(void (*callback)());
    void on_down_released_event(void (*callback)());
    void on_left_released_event(void (*callback)());
    void on_right_released_event(void (*callback)());
    
    //ABXY Events
    void on_a_pressed_event(void (*callback)());
    void on_b_pressed_event(void (*callback)());
    void on_x_pressed_event(void (*callback)());
    void on_y_pressed_event(void (*callback)());

    void on_a_released_event(void (*callback)());
    void on_b_released_event(void (*callback)());
    void on_x_released_event(void (*callback)());
    void on_y_released_event(void (*callback)());

    //Bumper Events
    void on_right_top_bumper_pressed_event(void (*callback)());
    void on_right_bottom_bumper_pressed_event(void (*callback)());
    void on_left_top_bumper_pressed_event(void (*callback)());
    void on_left_bottom_bumper_pressed_event(void (*callback)());

    void on_right_top_bumper_released_event(void (*callback)());
    void on_right_bottom_bumper_released_event(void (*callback)());
    void on_left_top_bumper_released_event(void (*callback)());
    void on_left_bottom_bumper_released_event(void (*callback)());

};


#endif // WHOOP_MOTOR_HPP

