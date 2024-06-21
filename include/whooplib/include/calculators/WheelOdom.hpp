/**
 * General-use odometry class with X_position, Y_position, and
 * orientation_rad being the relevant outputs. This works for both one
 * and two-tracker systems, and requires a gyro to determine the input angle.
 * Note: The gyro input is counter-clockwise-positive and provided in radians,
 * which standardizes the rotation. This odometry system is a modified version
 * adapted from the Jar-Template, specifically tailored for academic use.
 * Source: https://github.com/JacksonAreaRobotics/JAR-Template/blob/main/src/JAR-Template/odom.cpp
 */

#ifndef ODOM_HPP
#define ODOM_HPP

/**
 * Wheel Odometry Object
 */
class WheelOdom
{
private:
  float ForwardTracker_center_distance; // Horizontal distance from the robot's center to the forward wheel's sensor.
  float SidewaysTracker_center_distance; // Vertical distance from the robot's center to the sideways wheel's sensor.
  float ForwardTracker_position; // Current horizontal position of the forward tracker in meters.
  float SideWaysTracker_position; // Current vertical position of the sideways tracker in meters.
public:
  float X_position; // Field-centric X position of the robot in meters.
  float Y_position; // Field-centric Y position of the robot in meters.
  float orientation_rad; // Robot's orientation in radians, where 0 radians aligns with the positive Y-direction.

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
  void set_position(float X_position, float Y_position, float orientation_rad, float ForwardTracker_position, float SidewaysTracker_position);

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
  void update_position(float ForwardTracker_position, float SidewaysTracker_position, float orientation_rad);

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
  void set_physical_distances(float ForwardTracker_center_distance, float SidewaysTracker_center_distance);
};

#endif
