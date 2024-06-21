#ifndef WHOOP_DRIVETRAIN_HPP
#define WHOOP_DRIVETRAIN_HPP

#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/calculators/WheelOdom.hpp"
#include "vex.h"
#include <vector>
#include <memory>

/**
 * Enum representing the possible states of the drivetrain.
 */
enum drivetrainState{
    mode_disabled=1, // The drivetrain is disabled and not responsive to input.
    mode_autonomous=2, // The drivetrain is operating under autonomous control.
    mode_usercontrol=3 // The drivetrain is responsive to user control.
};

/**
 * Class responsible for managing the drivetrain of a robot, including motor control and state management.
 */
class WhoopDrivetrain : public ComputeNode  {
protected:
    // Upon initialization
    WhoopController* whoop_controller; // Controller object for receiving input from VEX controllers.
    std::unique_ptr<WhoopMotorGroup> left_motor_group; // Group of motors on the left side of the drivetrain.
    std::unique_ptr<WhoopMotorGroup> right_motor_group; // Group of motors on the right side of the drivetrain.

private:
    // Initializes motor groups directly from pointers.
    void init_motor_groups(WhoopMotorGroup* leftGroup, WhoopMotorGroup* rightGroup);
    // Initializes motor groups from a vector of motors.
    void init_motor_groups(const std::vector<WhoopMotor*>& leftMotors, const std::vector<WhoopMotor*>& rightMotors);
public:
    vex::mutex thread_lock;  // Mutex for synchronizing access to drivetrain components.
    drivetrainState drive_state = drivetrainState::mode_disabled; // Current operational state of the drivetrain.

    /**
     * Constructor for initializing the drivetrain with predefined motor groups.
     * @param controller Pointer to the controller managing user input.
     * @param leftMotorGroup Pointer to the motor group controlling the left side.
     * @param rightMotorGroup Pointer to the motor group controlling the right side.
     */
    WhoopDrivetrain(WhoopController* controller, WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup);

    /**
     * Constructor for initializing the drivetrain with a list of motors for each side.
     * @param controller Pointer to the controller managing user input.
     * @param leftMotors Vector of motors on the left side.
     * @param rightMotors Vector of motors on the right side.
     */
    WhoopDrivetrain(WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors); 

    /**
     * Constructor for initializing the drivetrain with predefined motor groups and a gear ratio.
     * @param gear_ratio Gear ratio affecting the torque and speed of the motors.
     * @param controller Pointer to the controller managing user input.
     * @param leftMotorGroup Pointer to the motor group controlling the left side.
     * @param rightMotorGroup Pointer to the motor group controlling the right side.
     */
    WhoopDrivetrain(double gear_ratio, WhoopController* controller, WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup);

    /**
     * Constructor for initializing the drivetrain with a list of motors for each side and a gear ratio.
     * @param gear_ratio Gear ratio affecting the torque and speed of the motors.
     * @param controller Pointer to the controller managing user input.
     * @param leftMotors Vector of motors on the left side.
     * @param rightMotors Vector of motors on the right side.
     */
    WhoopDrivetrain(double gear_ratio, WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors);

    /**
     * Sets the gear ratio multiplier for the drivetrain.
     * i.e. motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0 = 0.5
     * @param ratio The new gear ratio to apply.
     */
    void set_gear_ratio_mult(double ratio); // motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0

    /**
     * Sets the operational state of the drivetrain.
     * @param state The new state to set (disabled, autonomous, or user control).
     */
    void set_state(drivetrainState state);

protected:
    /**
     * Override of ComputeNode's __step method to update the drivetrain's operation each cycle.
     */
    void __step() override;  // Protected helper function for processing steps
};


#endif // WHOOP_DRIVETRAIN_HPP

