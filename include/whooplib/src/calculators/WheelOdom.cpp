/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WheelOdom.cpp                                             */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Wheel X, Y, Yaw Tracking                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/calculators/WheelOdom.hpp"
#include "whooplib/include/toolbox.hpp"

/**
 * Setter method for tracker center distances.
 * The forward tracker center distance is the horizontal distance from the 
 * center of the robot to the center of the wheel the sensor is measuring.
 * The sideways tracker center distance is the vertical distance from the 
 * center of the robot to the center of the sideways wheel being measured.
 * If there's really no sideways wheel we set the center distance to 0 and
 * pretend the wheel never spins, which is equivalent to a no-drift robot.
 * 
 * @param ForwardTracker_center_distance A horizontal distance to the wheel center in meters.
 * @param SidewaysTracker_center_distance A vertical distance to the wheel center in meters.
 */
void WheelOdom::set_physical_distances(double ForwardTracker_center_distance, double SidewaysTracker_center_distance){
  vex_mutex.lock();
  this->ForwardTracker_center_distance = ForwardTracker_center_distance;
  this->SidewaysTracker_center_distance = SidewaysTracker_center_distance;
  vex_mutex.unlock();
}

/**
 * Resets the position, including tracking wheels.
 * Position is field-centric, and orientation is such that 0 radians
 * is in the positive Y direction. Orientation can be provided with 
 * some flexibility, including less than 0 and greater than 2*pi.
 * 
 * @param X_position Field-centric x position of the robot.
 * @param Y_position Field-centric y position of the robot.
 * @param orientation_rad Field-centered, counter-clockwise-positive, orientation in radians.
 * @param ForwardTracker_position Current position of the sensor in meters.
 * @param SidewaysTracker_position Current position of the sensor in meters.
 */
void WheelOdom::set_position(double X_position, double Y_position, double orientation_rad, double ForwardTracker_position, double SidewaysTracker_position){
  vex_mutex.lock();
  this->ForwardTracker_position = ForwardTracker_position;
  this->SideWaysTracker_position = SidewaysTracker_position;
  this->X_position = X_position;
  this->Y_position = Y_position;
  this->orientation_rad = orientation_rad;
  vex_mutex.unlock();
}

/**
 * Does the odometry math to update position
 * Uses the Pilons arc method outlined here: https://wiki.purduesigbots.com/software/odometry
 * All the deltas are done by getting member variables and comparing them to 
 * the input. Ultimately this all works to update the public member variable
 * X_position. This function needs to be run at 200Hz or so for best results.
 * 
 * @param ForwardTracker_position Current position of the sensor in meters.
 * @param SidewaysTracker_position Current position of the sensor in meters.
 * @param orientation_rad Field-centered, counter-clockwise-positive, orientation in radians.
 */
void WheelOdom::_update_pose(double ForwardTracker_position, double SidewaysTracker_position, double orientation_rad){
    Forward_delta = ForwardTracker_position - this->ForwardTracker_position;
    Sideways_delta = SidewaysTracker_position - this->SideWaysTracker_position;
    orientation_delta_rad = orientation_rad - this->orientation_rad;

    vex_mutex.lock();
    this->ForwardTracker_position = ForwardTracker_position;
    this->SideWaysTracker_position = SidewaysTracker_position;
    this->orientation_rad = orientation_rad;
    vex_mutex.unlock();

    double delta_radius = 0.001; // Threshold for considering orientation change as negligible

    if (fabs(orientation_delta_rad) < delta_radius) {
        local_X_position = Sideways_delta;
        local_Y_position = Forward_delta;
    } else {
        double radius_forward = Forward_delta / orientation_delta_rad + ForwardTracker_center_distance;
        double radius_sideways = Sideways_delta / orientation_delta_rad + SidewaysTracker_center_distance;
        local_X_position = 2 * sin(orientation_delta_rad / 2) * radius_sideways; 
        local_Y_position = 2 * sin(orientation_delta_rad / 2) * radius_forward;
    }

    local_polar_angle = atan2(local_Y_position, local_X_position); 
    local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2)); 

    // Correct application of CCW rotation to global orientation
    global_polar_angle = local_polar_angle + this->orientation_rad + (orientation_delta_rad / 2);

    X_position_delta = local_polar_length * cos(global_polar_angle); 
    Y_position_delta = local_polar_length * sin(global_polar_angle);

    vex_mutex.lock();
    X_position += X_position_delta;
    Y_position += Y_position_delta;
    vex_mutex.unlock();
}
