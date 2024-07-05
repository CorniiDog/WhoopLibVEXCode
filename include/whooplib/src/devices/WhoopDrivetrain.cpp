/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDrivetrain.cpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Drivetrain for Controlling Chassis                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/devices/WhoopDrivetrain.hpp"
#include "whooplib/include/devices/WhoopOdomFusion.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <memory> // For std::unique_ptr

void WhoopDrivetrain::init_motor_groups(WhoopMotorGroup *leftGroup, WhoopMotorGroup *rightGroup)
{
    left_motor_group = std::make_unique<WhoopMotorGroup>(*leftGroup);
    right_motor_group = std::make_unique<WhoopMotorGroup>(*rightGroup);
}

void WhoopDrivetrain::init_motor_groups(const std::vector<WhoopMotor *> &leftMotors, const std::vector<WhoopMotor *> &rightMotors)
{
    left_motor_group = std::make_unique<WhoopMotorGroup>(leftMotors);
    right_motor_group = std::make_unique<WhoopMotorGroup>(rightMotors);
}

WhoopDrivetrain::WhoopDrivetrain(PursuitParams *default_pursuit_parameters, WhoopOdomFusion *odom_fusion, PoseUnits pose_units, WhoopController *controller, WhoopMotorGroup *leftMotorGroup, WhoopMotorGroup *rightMotorGroup)
    : whoop_controller(controller), pursuit_conductor(default_pursuit_parameters)
{
    init_motor_groups(leftMotorGroup, rightMotorGroup);
    this->odom_fusion = odom_fusion;
    this->pose_units = pose_units;
    this->default_pose_units = pose_units;
}

WhoopDrivetrain::WhoopDrivetrain(PursuitParams *default_pursuit_parameters, WhoopOdomFusion *odom_fusion, PoseUnits pose_units, WhoopController *controller, std::vector<WhoopMotor *> leftMotors, std::vector<WhoopMotor *> rightMotors)
    : whoop_controller(controller), pursuit_conductor(default_pursuit_parameters)
{
    init_motor_groups(leftMotors, rightMotors);
    this->odom_fusion = odom_fusion;
    this->pose_units = pose_units;
    this->default_pose_units = pose_units;
}

void WhoopDrivetrain::set_state(drivetrainState state)
{
    if (state == drivetrainState::mode_autonomous)
    {
        pose_units = default_pose_units;
    }
    drive_state = state;
}

void WhoopDrivetrain::drive_to_point(double x, double y)
{
    drive_to_point(x, y, -1);
}

void WhoopDrivetrain::drive_to_point(double x, double y, double timeout_seconds)
{
    // Converting from standardized meters to inches
    if (pose_units == PoseUnits::in_deg_ccw || pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_ccw || pose_units == PoseUnits::in_rad_cw)
    {
        x = to_meters(x); // (meters -> inches)
        y = to_meters(y); // (meters -> inches)
    }

    TwoDPose robot_pose = odom_fusion->get_pose_2d();
    double yaw = robot_pose.yaw;

    TwoDPose pose(x, y, yaw);
    pursuit_conductor.generate_path(robot_pose, pose, timeout_seconds);
    auton_traveling = true;
}

void WhoopDrivetrain::drive_to_pose(double x, double y, double yaw)
{
    drive_to_pose(x, y, yaw, -1, -1);
}

void WhoopDrivetrain::drive_to_pose(double x, double y, double yaw, double timeout_seconds)
{
    drive_to_pose(x, y, yaw, timeout_seconds, -1);
}

void WhoopDrivetrain::drive_to_pose(double x, double y, double yaw, double timeout_seconds, double turning_radius)
{
    // Converting from inches to standardized meters
    if (pose_units == PoseUnits::in_deg_ccw || pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_ccw || pose_units == PoseUnits::in_rad_cw)
    {
        x = to_meters(x);                           // (inches -> meters)
        y = to_meters(y);                           // (inches -> meters)
        turning_radius = to_meters(turning_radius); // (inches -> meters)
    }

    // Converting from degrees to standardized radians
    if (pose_units == PoseUnits::m_deg_cw || pose_units == PoseUnits::m_deg_ccw || pose_units == PoseUnits::in_deg_ccw || pose_units == PoseUnits::in_deg_cw)
    {
        yaw = to_rad(yaw); // (degrees -> radians)
    }

    // Flipping from clockwise to standardized counter-clockwise
    if (pose_units == PoseUnits::m_deg_cw || pose_units == PoseUnits::m_rad_cw || pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_cw)
    {
        yaw *= -1; // (clockwise-positive -> counter-clockwise-positive)
    }

    TwoDPose pose(x, y, yaw);
    pursuit_conductor.generate_path(odom_fusion->get_pose_2d(), pose, timeout_seconds, turning_radius);
    auton_traveling = true;
}

// This is the protocol for calibrating the drivetrain while in a disabled state.
void WhoopDrivetrain::run_disabled_calibration_protocol()
{
    if (drive_state == drivetrainState::mode_disabled)
    {
        if (odom_fusion->is_moving())
        {
            needs_calibration = true;
            calibration_timer = 0;
            if (moved_one_time_notif)
            {
                whoop_controller->notify("Robot Moved");
                moved_one_time_notif = false;
            }
        }
        else if (needs_calibration)
        { // Stationary and needs calibration
            calibration_timer += 20;
            if (calibration_timer > time_until_calibration)
            { // If stationary for more than period of time (like 500 milliseconds) then calibrate
                calibrate();
                needs_calibration = false;
                moved_one_time_notif = true;
            }
        }
    }
}

void WhoopDrivetrain::calibrate()
{
    whoop_controller->notify("Calibrating Dont Move");
    odom_fusion->calibrate();
    whoop_controller->notify("Calibration Finished.", 2);
}

/**
 * Gets the x, y, z, pitch, yaw, roll of the robot
 * @return Pose object. With Pose.x, Pose.y, etc...
 */
Pose WhoopDrivetrain::get_pose()
{
    Pose p = odom_fusion->get_pose();

    // Converting from standardized meters to inches
    if (pose_units == PoseUnits::in_deg_ccw || pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_ccw || pose_units == PoseUnits::in_rad_cw)
    {
        p.x = to_inches(p.x); // (meters -> inches)
        p.y = to_inches(p.y); // (meters -> inches)
        p.z = to_inches(p.z); // (meters -> inches)
    }

    // Converting from standardized radians to degrees
    if (pose_units == PoseUnits::m_deg_cw || pose_units == PoseUnits::m_deg_ccw || pose_units == PoseUnits::in_deg_ccw || pose_units == PoseUnits::in_deg_cw)
    {
        p.pitch = to_deg(p.pitch); // (radians -> degrees)
        p.roll = to_deg(p.roll);   // (radians -> degrees)
        p.yaw = to_deg(p.yaw);     // (radians -> degrees)
    }

    // Flipping from standardized counter-clockwise to clockwise
    if (pose_units == PoseUnits::m_deg_cw || pose_units == PoseUnits::m_rad_cw || pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_cw)
    {
        p.yaw *= -1; // (counter-clockwise-positive -> degrees clockwise-positive)
    }

    return p;
}

/**
 * Sets the pose of the robot
 * @param x_in the location, units configured upon initialization, parallel to the width of the driver station
 * @param y_in the location, units configured upon initialization, perpendicular to the width of the driver station
 * @param yaw_deg the rotation, units configured upon initialization
 */
void WhoopDrivetrain::set_pose(double x, double y, double yaw)
{

    // Converting from inches to standardized meters (in -> m)
    if (pose_units == PoseUnits::in_deg_ccw || pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_ccw || pose_units == PoseUnits::in_rad_cw)
    {
        x = to_meters(x); // (inches -> meters)
        y = to_meters(y); // (inches -> meters)
    }

    // Converting from degrees to standardized radians (deg -> rad)
    if (pose_units == PoseUnits::m_deg_cw || pose_units == PoseUnits::m_deg_ccw || pose_units == PoseUnits::in_deg_ccw || pose_units == PoseUnits::in_deg_cw)
    {
        yaw = to_rad(yaw); // (degrees -> radians)
    }

    // Flipping from clockwise to standardized counter-clockwise (cw -> ccw)
    if (pose_units == PoseUnits::m_deg_cw || pose_units == PoseUnits::m_rad_cw || pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_cw)
    {
        yaw *= -1; // (counter-clockwise-positive -> clockwise-positive)
    }

    odom_fusion->tare(x, y, yaw);
}

void WhoopDrivetrain::step_usercontrol()
{
    switch (whoop_controller->joystick_mode)
    {
    case joystickMode::joystickmode_tank:
        left_motor_group->spin_percentage(whoop_controller->get_left_joystick_y());
        right_motor_group->spin_percentage(whoop_controller->get_right_joystick_y());
        break;
    case joystickMode::joystickmode_split_arcade:
        left_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() + whoop_controller->get_right_joystick_x());
        right_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() - whoop_controller->get_right_joystick_x());
        break;
    case joystickMode::joystickmode_left_arcade:
        left_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() + whoop_controller->get_left_joystick_x());
        right_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() - whoop_controller->get_left_joystick_x());
        break;
    case joystickMode::joystickmode_right_arcade:
        left_motor_group->spin_percentage(whoop_controller->get_right_joystick_y() + whoop_controller->get_right_joystick_x());
        right_motor_group->spin_percentage(whoop_controller->get_right_joystick_y() - whoop_controller->get_right_joystick_x());
        break;
    }
}

void WhoopDrivetrain::step_disabled()
{
    left_motor_group->spin(0);
    right_motor_group->spin(0);
    run_disabled_calibration_protocol();
}

void WhoopDrivetrain::step_autonomous()
{
    if (auton_traveling)
    {
        PursuitResult result = pursuit_conductor.step(odom_fusion->get_pose_2d());
        if (result.is_completed)
        {   
            Brain.Screen.clearLine(1);
            Brain.Screen.setCursor(1,1);
            Brain.Screen.print("Completed");
            auton_traveling = false;
            return;
        }

        if (!result.is_valid)
        {
            Brain.Screen.clearLine(1);
            Brain.Screen.setCursor(1,1);
            Brain.Screen.print("Not accepting valid");
            return;
        }
        Brain.Screen.clearLine(1);
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Running");

        //left_motor_group->spin(result.forward_power + std::min(-result.steering_power, 0.0));
        //right_motor_group->spin(result.forward_power + std::min(result.steering_power, 0.0));
    }
    else
    {
        left_motor_group->spin(0);
        right_motor_group->spin(0);
    }
}

void WhoopDrivetrain::__step()
{
    odom_fusion->__step(); // Step odometry fusion module

    switch (drive_state)
    {
    case drivetrainState::mode_usercontrol:
        step_usercontrol();
        break;
    case drivetrainState::mode_autonomous:
        step_autonomous();
        break;
    case drivetrainState::mode_disabled:
        step_disabled();
        break;
    }
}

/**
 * Gets units that the odometry is using
 * @returns units desciber, as a string
 */
std::string WhoopDrivetrain::get_units_str()
{
    switch (pose_units)
    {
    case m_deg_cw:
        return "m_deg_cw";
    case m_deg_ccw:
        return "m_deg_ccw";
    case m_rad_cw:
        return "m_rad_cw";
    case m_rad_ccw:
        return "m_rad_ccw";
    case in_deg_cw:
        return "in_deg_cw";
    case in_deg_ccw:
        return "in_deg_ccw";
    case in_rad_cw:
        return "in_rad_cw";
    case in_rad_ccw:
        return "in_rad_ccw";
    default:
        return "Unknown units";
    }
}

/**
 * Gets units that the odometry is using
 * @returns units desciber, as a Pose/UnitsObject
 */
PoseUnits WhoopDrivetrain::get_units()
{
    return pose_units;
}

void WhoopDrivetrain::set_pose_units(PoseUnits units)
{
    pose_units = units;
}