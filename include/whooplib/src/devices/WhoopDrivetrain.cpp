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

void WhoopDrivetrain::drive_to_point(double x, double y, waitUntilCompleted wait_until_completed)
{
    drive_to_point(x, y, -1, wait_until_completed);
}

void WhoopDrivetrain::drive_to_point(double x, double y, double timeout_seconds, waitUntilCompleted wait_until_completed)
{
    Pose robot_pose_respected_units = get_pose();
    drive_through_path({{x, y, robot_pose_respected_units.yaw}}, timeout_seconds, -1, wait_until_completed);
}

void WhoopDrivetrain::drive_to_pose(double x, double y, double yaw, waitUntilCompleted wait_until_completed)
{
    drive_to_pose(x, y, yaw, -1, -1, wait_until_completed);
}

void WhoopDrivetrain::drive_to_pose(double x, double y, double yaw, double timeout_seconds, waitUntilCompleted wait_until_completed)
{
    drive_to_pose(x, y, yaw, timeout_seconds, -1, wait_until_completed);
}

void WhoopDrivetrain::drive_to_pose(double x, double y, double yaw, double timeout_seconds, double turning_radius, waitUntilCompleted wait_until_completed)
{
    drive_through_path({{x, y, yaw}}, timeout_seconds, turning_radius, wait_until_completed);
}

void WhoopDrivetrain::drive_through_path(std::vector<std::vector<double>> waypoints, waitUntilCompleted wait_until_completed)
{
    drive_through_path(waypoints, -1, -1, wait_until_completed);
}

void WhoopDrivetrain::drive_through_path(std::vector<std::vector<double>> waypoints, double timeout_seconds, waitUntilCompleted wait_until_completed)
{
    drive_through_path(waypoints, -1, -1, wait_until_completed);
}

void WhoopDrivetrain::reverse_to_point(double x, double y, waitUntilCompleted wait_until_completed){
    reverse_to_point(x, y, -1, wait_until_completed);
}

void WhoopDrivetrain::reverse_to_point(double x, double y, double timeout_seconds, waitUntilCompleted wait_until_completed){
    request_reverse = true;
    drive_to_point(x, y, timeout_seconds, wait_until_completed);
}

void WhoopDrivetrain::reverse_to_pose(double x, double y, double yaw, waitUntilCompleted wait_until_completed){
    reverse_to_pose(x, y, yaw, -1, -1, wait_until_completed);
}

void WhoopDrivetrain::reverse_to_pose(double x, double y, double yaw, double timeout_seconds, waitUntilCompleted wait_until_completed){
    reverse_to_pose(x, y, yaw, timeout_seconds, -1, wait_until_completed);
}

void WhoopDrivetrain::reverse_to_pose(double x, double y, double yaw, double timeout_seconds, double turning_radius, waitUntilCompleted wait_until_completed){
    request_reverse = true;
    drive_to_pose(x, y, yaw, timeout_seconds, turning_radius, wait_until_completed);
}

void WhoopDrivetrain::reverse_through_path(std::vector<std::vector<double>> waypoints, waitUntilCompleted wait_until_completed){
    reverse_through_path(waypoints, -1, -1, wait_until_completed);
}

void WhoopDrivetrain::reverse_through_path(std::vector<std::vector<double>> waypoints, double timeout_seconds, waitUntilCompleted wait_until_completed){
    reverse_through_path(waypoints, timeout_seconds, -1, wait_until_completed);
}

void WhoopDrivetrain::reverse_through_path(std::vector<std::vector<double>> waypoints, double timeout_seconds, double turning_radius, waitUntilCompleted wait_until_completed){
    request_reverse = true;
    drive_through_path(waypoints, timeout_seconds, turning_radius, wait_until_completed);
}

void WhoopDrivetrain::drive_through_path(std::vector<std::vector<double>> waypoints, double timeout_seconds, double turning_radius, waitUntilCompleted wait_until_completed)
{
    std::cout << "Generating Path" << std::endl;
    if(request_reverse){
        request_reverse = false;
        auton_reverse = true;
    }
    else{
        auton_reverse = false;
    }

    // Ensure that waypoints are 2 or greater
    size_t waypoints_size = waypoints.size();
    if (waypoints_size < 1)
    {
        Brain.Screen.print("A path requires at least 1 waypoint");
        std::cout << "A path requires at least 1 waypoint" << std::endl;
    }

    // First check the validity of the list of lists to be appropriately structured
    size_t size_of;
    for (size_t i = 0; i < waypoints_size; i++)
    {
        size_of = waypoints[i].size();
        if (size_of != 2 && size_of != 3)
        { // If regular waypoint, must be either 2 or 3 variables
            Brain.Screen.print("Waypoints must consist of either 3 variables {x, y, yaw}, or 2 variables {x, y}");
            std::cout << "Waypoints must consist of either 3 variables {x, y, yaw}, or 2 variables {x, y}" << std::endl;
        }
    }

    // Converting from inches to standardized meters
    bool convert_to_meters = false;
    if (pose_units == PoseUnits::in_deg_ccw || pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_ccw || pose_units == PoseUnits::in_rad_cw)
    {
        convert_to_meters = true;
        turning_radius = to_meters(turning_radius); // (inches -> meters)
    }

    bool convert_to_radians = false;
    // Converting from degrees to standardized radians
    if (pose_units == PoseUnits::m_deg_cw || pose_units == PoseUnits::m_deg_ccw || pose_units == PoseUnits::in_deg_ccw || pose_units == PoseUnits::in_deg_cw)
    {
        convert_to_radians = true;
    }

    bool reverse_rotation = false;
    // Flipping from clockwise to standardized counter-clockwise
    if (pose_units == PoseUnits::m_deg_cw || pose_units == PoseUnits::m_rad_cw || pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_cw)
    {
        reverse_rotation = true;
    }

    // Create a new waypoints list
    std::vector<std::vector<double>> validated_waypoints;

    // Add start pose to the beginning
    TwoDPose start_pose = odom_fusion->get_pose_2d();
    validated_waypoints.push_back({start_pose.x, start_pose.y, start_pose.yaw});

    TwoDPose target_pose;

    // Adjust to valid sizes with respect to configured units, then add to validated
    for (size_t i = 0; i < waypoints_size; i++)
    {
        size_of = waypoints[i].size();

        std::vector<double> waypoint_data;
        waypoint_data.push_back(waypoints[i][0]);
        waypoint_data.push_back(waypoints[i][1]);

        // Converting from inches to standardized meters
        if (convert_to_meters)
        {
            waypoint_data[0] = to_meters(waypoint_data[0]); // (inches -> meters)
            waypoint_data[1] = to_meters(waypoint_data[1]); // (inches -> meters)
        }

        if (size_of == 3) // If contains yaw
        {
            waypoint_data.push_back(waypoints[i][2]);

            if (convert_to_radians)
                waypoint_data[2] = to_rad(waypoint_data[2]); // (degrees -> radians)

            if (reverse_rotation)
                waypoint_data[2] *= -1; // (clockwise-positive -> counter-clockwise-positive)

        }
        validated_waypoints.push_back(waypoint_data);

        // If last element, we need to document it
        if (i == waypoints_size - 1)
        {
            if (size_of == 3)
            {
                target_pose = TwoDPose(waypoint_data[0], waypoint_data[1], waypoint_data[2]);
            }
            else
            { // 2
                // Yoink yaw from start pose with its own pose
                target_pose = TwoDPose(waypoint_data[0], waypoint_data[1], validated_waypoints[0][2]);
            }
        }
    }

    // If in reverse, flip the yaw of the first waypoint
    if(auton_reverse){
        validated_waypoints[0][2] = normalize_angle(validated_waypoints[0][2] + M_PI);
    }

    pursuit_conductor.generate_path(validated_waypoints, timeout_seconds, turning_radius);

    auton_traveling = true;

    if (wait_until_completed == waitUntilCompleted::yes_wait)
    {
        this->wait_until_completed();
    }

    // Flip target pose so that the system knows the direction the robot is actually looking
    if(auton_reverse){
        target_pose.yaw = normalize_angle(target_pose.yaw + M_PI);
    }

    last_desired_position = desired_position;
    desired_position = target_pose;
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

void WhoopDrivetrain::wait_until_completed()
{
    while (auton_traveling)
    {
        wait(5, msec);
    }
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
        TwoDPose robot_pose = odom_fusion->get_pose_2d();
        if(auton_reverse){
            robot_pose.yaw = normalize_angle(robot_pose.yaw + M_PI);
        }

        pursuit_result = pursuit_conductor.step(robot_pose);
        if(temp_disable){
            left_motor_group->spin(0);
            right_motor_group->spin(0);
            return;
        }

        if (pursuit_result.is_completed)
        {
            auton_traveling = false;
            return;
        }

        if (!pursuit_result.is_valid)
        {
            auton_traveling = false;
            return;
        }

        if (pursuit_conductor.forward_pid.is_settled() || pursuit_result.suggest_point_turn)
        {
            if(auton_reverse){
                left_motor_group->spin(-pursuit_result.forward_power - pursuit_result.steering_power / 1.5);
                right_motor_group->spin(-pursuit_result.forward_power + pursuit_result.steering_power / 1.5);
            }
            else{
                left_motor_group->spin(pursuit_result.forward_power - pursuit_result.steering_power / 1.5);
                right_motor_group->spin(pursuit_result.forward_power + pursuit_result.steering_power / 1.5);
            }   
        }
        else
        {
            if(auton_reverse){
                left_motor_group->spin(-pursuit_result.forward_power + std::max(-pursuit_result.steering_power, 0.0));
                right_motor_group->spin(-pursuit_result.forward_power + std::max(pursuit_result.steering_power, 0.0));
            }
            else{
                left_motor_group->spin(pursuit_result.forward_power + std::min(-pursuit_result.steering_power, 0.0));
                right_motor_group->spin(pursuit_result.forward_power + std::min(pursuit_result.steering_power, 0.0));
            }
        }
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

void WhoopDrivetrain::display_map()
{
    double m_to_pixels = 30;
    double screen_offset = 3.6 * 30;
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(color(0, 255, 0));
    Brain.Screen.setFillColor(color::transparent);
    if (pursuit_conductor.enabled)
    {
        int size = pursuit_conductor.pursuit_path.pursuit_points.size();
        for (int i = 0; i < size; ++i)
        {
            barebonesPose pose = pursuit_conductor.pursuit_path.pursuit_points[i];
            TwoDPose pose_on_screen(pose.x * m_to_pixels + screen_offset, -pose.y * m_to_pixels + screen_offset, pose.yaw);
            Brain.Screen.drawPixel(pose_on_screen.x, pose_on_screen.y);
        }
    }

    // Display robot position
    TwoDPose robot_pose = odom_fusion->get_pose_2d();
    TwoDPose robot_pose_on_screen(robot_pose.x * m_to_pixels + screen_offset, -robot_pose.y * m_to_pixels + screen_offset, robot_pose.yaw);
    Brain.Screen.setPenColor(color(255, 0, 0));
    Brain.Screen.drawCircle(robot_pose_on_screen.x, robot_pose_on_screen.y, 2);
    Brain.Screen.drawLine(robot_pose_on_screen.x, robot_pose_on_screen.y, robot_pose_on_screen.x + 10 * cos(robot_pose_on_screen.yaw), robot_pose_on_screen.y - 10 * sin(robot_pose_on_screen.yaw));

    // Draw robot lookahead radius
    Brain.Screen.setPenColor(color(255, 150, 0));
    Brain.Screen.drawCircle(robot_pose_on_screen.x, robot_pose_on_screen.y, pursuit_conductor.pursuit_path.lookahead_distance * m_to_pixels);

    // Draw lookahead point
    Brain.Screen.setPenColor(color(255, 255, 255));
    barebonesPose lookahead_pos = pursuit_conductor.pursuit_path.lookahead_pos;
    TwoDPose lookahead_pose_on_screen(lookahead_pos.x * m_to_pixels + screen_offset, -lookahead_pos.y * m_to_pixels + screen_offset, 0);
    Brain.Screen.drawCircle(lookahead_pose_on_screen.x, lookahead_pose_on_screen.y, 2);

    Brain.Screen.setPenColor(color(255, 255, 255));

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Distance: %.1f | Distance Power %.1f", pursuit_result.distance, pursuit_result.forward_power);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Turn: %.1f | Turn Power: %.1f", pursuit_result.steering_angle, pursuit_result.steering_power);

    TwoDPose current_pose = odom_fusion->get_pose_2d();
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Position: %.2f %.2f %.2f", current_pose.x, current_pose.y, current_pose.yaw);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Lookahead Point: %.2f %.2f %.2f", get_units_str().c_str(), lookahead_pos.x, lookahead_pos.y, lookahead_pos.yaw);
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