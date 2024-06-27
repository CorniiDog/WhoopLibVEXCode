/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDrivetrain.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Drivetrain for Controlling Chassis                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_DRIVETRAIN_HPP
#define WHOOP_DRIVETRAIN_HPP

#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/calculators/WheelOdom.hpp"
#include "whooplib/include/devices/WhoopOdomFusion.hpp"
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
 * Enum representing the pose units for movement
 */
enum PoseUnits{
    m_deg_cw, // meters and degrees clockwise
    m_deg_ccw, // meters and degrees counter-clockwise
    m_rad_cw, // meters and radians clockwise
    m_rad_ccw, // meters and radians counter-clockwise
    in_deg_cw, // inches and degrees clockwise
    in_deg_ccw, // inches and degrees counter-clockwise
    in_rad_cw, // inches and radians clockwise
    in_rad_ccw, // inches and radians counter-clockwise
};

/**
 * Class responsible for managing the drivetrain of a robot, including motor control and state management.
 */
class WhoopDrivetrain : public ComputeNode  {
private:
    // Calibration protocol settings
    double time_until_calibration = 1000; // ms
    // Calibration protocol Modifiables
    bool needs_calibration = true;
    double calibration_timer = 0;
    bool moved_one_time_notif = false;
    // This runs the calibration protocol for the drivetrain
    void run_disabled_calibration_protocol();
protected:
    // Upon initialization
    WhoopController* whoop_controller; // Controller object for receiving input from VEX controllers.
    std::unique_ptr<WhoopMotorGroup> left_motor_group; // Group of motors on the left side of the drivetrain.
    std::unique_ptr<WhoopMotorGroup> right_motor_group; // Group of motors on the right side of the drivetrain.
    WhoopOdomFusion* odom_fusion; // Group of motors on the right side of the drivetrain.
    PoseUnits pose_units;

    bool drive_calibrated = false;
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
     * @param odom_fusion The odometry fusion system for pose estimation
     * @param pose_units Configure the units for the odometry. "m_deg_cw" means "meters, degrees, clockwise-positive yaw", "in_deg_ccw" means "inches, degrees, counter-clockwise-positive yaw", and so forth.
     * @param controller Pointer to the controller managing user input.
     * @param leftMotorGroup Pointer to the motor group controlling the left side.
     * @param rightMotorGroup Pointer to the motor group controlling the right side.
     */
    WhoopDrivetrain(WhoopOdomFusion* odom_fusion, PoseUnits pose_units, WhoopController* controller, WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup);

    /**
     * Constructor for initializing the drivetrain with a list of motors for each side.
     * @param odom_fusion The odometry fusion system for pose estimation
     * @param pose_units Configure the units for the odometry. "m_deg_cw" means "meters, degrees, clockwise-positive yaw", "in_deg_ccw" means "inches, degrees, counter-clockwise-positive yaw", and so forth.
     * @param controller Pointer to the controller managing user input.
     * @param leftMotors Vector of motors on the left side.
     * @param rightMotors Vector of motors on the right side.
     */
    WhoopDrivetrain(WhoopOdomFusion* odom_fusion, PoseUnits pose_units, WhoopController* controller, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors); 

    /**
     * Sets the operational state of the drivetrain.
     * @param state The new state to set (disabled, autonomous, or user control).
     */
    void set_state(drivetrainState state);

    /**
     * Calibrates the robot drivetrain
     */
    void calibrate();

    /**
     * Gets the x, y, z, pitch, yaw, roll of the robot
     * @return Pose object. With Pose.x, Pose.y, etc...
     */
    Pose get_pose();

    /**
     * Sets the pose of the robot
     * @param x_in the location, in inches, parallel to the width of the driver station
     * @param y_in the location, in inches, perpendicular to the width of the driver station
     * @param yaw_deg the rotation, clockwise-positive, in degrees
     */
    void set_pose(double x_in, double y_in, double yaw_deg);

protected:
    /**
     * Override of ComputeNode's __step method to update the drivetrain's operation each cycle.
     */
    void __step() override;  // Protected helper function for processing steps
};


#endif // WHOOP_DRIVETRAIN_HPP

