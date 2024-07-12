/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopController.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Controller With Additional Functions              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_CONTROLLER_HPP
#define WHOOP_CONTROLLER_HPP

#include "vex.h"
#include <vector>
#include <functional>
#include "whooplib/include/nodes/NodeManager.hpp"

/**
 * Enumerates the available joystick control modes.
 */
enum joystickMode
{
    joystickmode_tank = 1,
    joystickmode_split_arcade = 2,
    joystickmode_left_arcade = 3,
    joystickmode_right_arcade = 4
};

/**
 * Controls and manages inputs from a VEX controller.
 */
class WhoopController : public ComputeNode
{
public:
    vex::controller vex_controller; // Instance of VEX controller.
    joystickMode joystick_mode;     // Current joystick mode.

    int time_left_to_clear = 0;

    /**
     * Constructor that initializes the controller with a specific joystick mode.
     * @param mode The joystick mode to be used.
     */
    WhoopController(joystickMode mode = joystickMode::joystickmode_split_arcade);

    /**
     * Constructor that initializes the controller with a specific joystick mode and controller type.
     * @param mode The joystick mode to be used.
     * @param controller_type The type of controller (primary or partner).
     */
    WhoopController(joystickMode mode, vex::controllerType controller_type);

    /**
     * Retrieves the horizontal axis value of the left joystick.
     * @return The x-coordinate value from the left joystick [-100, 100].
     */
    double get_left_joystick_x();

    /**
     * Retrieves the vertical axis value of the left joystick.
     * @return The y-coordinate value from the left joystick [-100, 100].
     */
    double get_left_joystick_y();

    /**
     * Retrieves the horizontal axis value of the right joystick.
     * @return The x-coordinate value from the right joystick [-100, 100].
     */
    double get_right_joystick_x();

    /**
     * Retrieves the vertical axis value of the right joystick.
     * @return The y-coordinate value from the right joystick [-100, 100].
     */
    double get_right_joystick_y();

    /**
     * Notifies for a set period of time
     * @param message The message to send the notification
     * @param duration_seconds The duration to display the message
     */
    void notify(std::string message, double duration_seconds = 5);

    // UDLR
    bool up_pressing();
    bool down_pressing();
    bool left_pressing();
    bool right_pressing();

    // ABXY
    bool a_pressing();
    bool b_pressing();
    bool x_pressing();
    bool y_pressing();

    // Bumpers
    bool right_top_bumper_pressing();
    bool right_bottom_bumper_pressing();
    bool left_top_bumper_pressing();
    bool left_bottom_bumper_pressing();

    // UDLR Events
    void on_up_pressed_event(void (*callback)());
    void on_down_pressed_event(void (*callback)());
    void on_left_pressed_event(void (*callback)());
    void on_right_pressed_event(void (*callback)());

    void on_up_released_event(void (*callback)());
    void on_down_released_event(void (*callback)());
    void on_left_released_event(void (*callback)());
    void on_right_released_event(void (*callback)());

    // ABXY Events
    void on_a_pressed_event(void (*callback)());
    void on_b_pressed_event(void (*callback)());
    void on_x_pressed_event(void (*callback)());
    void on_y_pressed_event(void (*callback)());

    void on_a_released_event(void (*callback)());
    void on_b_released_event(void (*callback)());
    void on_x_released_event(void (*callback)());
    void on_y_released_event(void (*callback)());

    // Bumper Events
    void on_right_top_bumper_pressed_event(void (*callback)());
    void on_right_bottom_bumper_pressed_event(void (*callback)());
    void on_left_top_bumper_pressed_event(void (*callback)());
    void on_left_bottom_bumper_pressed_event(void (*callback)());

    void on_right_top_bumper_released_event(void (*callback)());
    void on_right_bottom_bumper_released_event(void (*callback)());
    void on_left_top_bumper_released_event(void (*callback)());
    void on_left_bottom_bumper_released_event(void (*callback)());

public:
    void __step() override; // Protected helper function for processing steps
};

#endif // WHOOP_CONTROLLER_HPP
