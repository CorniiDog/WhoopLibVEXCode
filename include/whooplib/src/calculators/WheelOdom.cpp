/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WheelOdom.cpp                                             */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Wheel X, Y, Yaw Tracking                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/WheelOdom.hpp"
#include "whooplib/include/toolbox.hpp"

void WheelOdom::set_position(double x, double y, double orientation) {
    X_position = x;
    Y_position = y;
    orientation_rad = orientation;
    tare_angle = orientation;  // Set the tare angle to the initial orientation

    // Normalize the orientation_rad
    while (orientation_rad > M_PI) orientation_rad -= 2 * M_PI;
    while (orientation_rad < -M_PI) orientation_rad += 2 * M_PI;
}

void WheelOdom::set_physical_distances(double forward_distance, double sideways_distance) {
    forward_tracker_center_distance = forward_distance;
    sideways_tracker_center_distance = sideways_distance;
}

void WheelOdom::update_pose(double forward_tracker_pos, double sideways_tracker_pos, double gyro_angle_rad) {
    double delta_forward = forward_tracker_pos - last_forward_tracker_pos;
    double delta_sideways = sideways_tracker_pos - last_sideways_tracker_pos;

    last_forward_tracker_pos = forward_tracker_pos;
    last_sideways_tracker_pos = sideways_tracker_pos;

    double delta_theta = gyro_angle_rad - orientation_rad;

    // Normalize delta_theta to be within the range -pi to pi
    delta_theta = normalize_angle(delta_theta);

    orientation_rad += delta_theta;
    orientation_rad = normalize_angle(orientation_rad);

    if (fabs(delta_theta) > 0) {  // Arc movement
        double radius = delta_forward / delta_theta;
        double icc_x = X_position - (radius + sideways_tracker_center_distance) * sin(orientation_rad);
        double icc_y = Y_position + (radius + sideways_tracker_center_distance) * cos(orientation_rad);

        X_position = cos(delta_theta) * (X_position - icc_x) - sin(delta_theta) * (Y_position - icc_y) + icc_x;
        Y_position = sin(delta_theta) * (X_position - icc_x) + cos(delta_theta) * (Y_position - icc_y) + icc_y;
    } else {  // Straight line approximation
        X_position += delta_sideways * cos(orientation_rad + M_PI / 2) - delta_sideways * sin(orientation_rad + M_PI / 2);
        Y_position += delta_forward * sin(orientation_rad) + delta_forward * cos(orientation_rad);
    }
}