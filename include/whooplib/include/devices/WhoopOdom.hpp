/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopOdom.hpp                                             */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Odometry Module for Pose Estimation                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_ODOM_HPP
#define WHOOP_ODOM_HPP

#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/devices/WhoopInertial.hpp"
#include "whooplib/include/devices/WhoopVision.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/calculators/WheelOdom.hpp"
#include "vex.h"
#include <vector>
#include <memory>

/**
 * Class responsible for managing the drivetrain of a robot, including motor control and state management.
 */
class WhoopOdom : public ComputeNode {
protected:
    // Upon initialization
    std::unique_ptr<WhoopMotorGroup> left_motor_group; // Group of motors on the left side of the drivetrain.
    std::unique_ptr<WhoopMotorGroup> right_motor_group; // Group of motors on the right side of the drivetrain.
    WhoopInertial* inertial_sensor;

private:
    // Initializes motor groups directly from pointers.
    void init_motor_groups(WhoopMotorGroup* leftGroup, WhoopMotorGroup* rightGroup);
    // Initializes motor groups from a vector of motors.
    void init_motor_groups(const std::vector<WhoopMotor*>& leftMotors, const std::vector<WhoopMotor*>& rightMotors);
public:
    vex::mutex thread_lock;  // Mutex for synchronizing access to odometry components.

    /**
     * Constructor for initializing the drivetrain with predefined motor groups and a gear ratio.
     * @param wheel_diameter_meters The diameter of the wheel in meters i.e. 0.08255 for 3.25 inch wheels
     * @param gear_ratio Gear ratio affecting the torque and speed of the motors.
     * @param leftMotorGroup Pointer to the motor group controlling the left side.
     * @param rightMotorGroup Pointer to the motor group controlling the right side.
     */
    WhoopOdom(double wheel_diameter_meters, double gear_ratio, WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup);

    /**
     * Constructor for initializing the drivetrain with a list of motors for each side and a gear ratio.
     * @param wheel_diameter_meters The diameter of the wheel in meters i.e. 0.08255 for 3.25 inch wheels
     * @param gear_ratio Gear ratio affecting the torque and speed of the motors.
     * @param leftMotors Vector of motors on the left side.
     * @param rightMotors Vector of motors on the right side.
     */
    WhoopOdom(double wheel_diameter_meters, double gear_ratio, std::vector<WhoopMotor*> leftMotors, std::vector<WhoopMotor*> rightMotors);

    /**
     * Sets the gear ratio multiplier for the drivetrain.
     * i.e. motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0 = 0.5
     * @param ratio The new gear ratio to apply.
     */
    void set_gear_ratio_mult(double ratio); // motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0

    /**
     * Sets the wheel diameter multiplier for the drivetrain, in meters
     * @param diameter_meters The wheel diameter in meters (i.e. 0.08255 for 3.25" wheels)
     */
    void set_wheel_diameter(double diameter_meters);

    // Taring (resetting) methods for the pose estimation.
    void tare(double x, double y, double z, double pitch, double yaw, double roll);
    void tare(double x, double y, double yaw, tare_remaining_0 tare_rest_to_zero);
    void tare(double x, double y, double yaw);
    void tare();
    
    /**
     * Retrieves the corrected and computed pose.
     * @return The current pose of the system.
     */
    Pose get_pose();
protected:
    /**
     * Override of ComputeNode's __step method to update the drivetrain's operation each cycle.
     */
    void __step() override;  // Protected helper function for processing steps
};


#endif // WHOOP_DRIVETRAIN_HPP

