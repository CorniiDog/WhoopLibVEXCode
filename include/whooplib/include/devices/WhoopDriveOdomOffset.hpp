/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDriveOdomOffset.hpp                                  */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Odometry Offset Module for Pose Estimation                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_DRIVE_ODOM_OFFSET_HPP
#define WHOOP_DRIVE_ODOM_OFFSET_HPP

#include "whooplib/include/devices/WhoopDriveOdomUnit.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/calculators/TwoDPose.hpp"
#include "vex.h"
#include <vector>
#include <memory>



/**
 * Class responsible for managing the odometry unit.
 */
class WhoopDriveOdomOffset : public ComputeNode {
public:
    WhoopDriveOdomUnit* odom_unit;

    TwoDPose pose = TwoDPose(0,0,0);
    TwoDPose last_pose = TwoDPose(0,0,0);
    TwoDPose offset;
    vex::mutex thread_lock;  // Mutex for synchronizing access to odometry components.

    /**
     * Constructor for Drive Odom Offset.
     * The odom unit center is the virtual intercept of the perpendicular faces of the odometry trackers.
     * Visual Representation of Odom Location: https://imgur.com/x8ObCIG
     * @param odom_unit A pointer to the odometry unit
     * @param x_offset The x offset of the odom unit from the center of the robot (positive implies a shift right from the center of the robot).
     * @param y_offset The y offset of the odom unit from the center of the robot (positive implies a shift forward from the center of the robot).
     */
    WhoopDriveOdomOffset(WhoopDriveOdomUnit* odom_unit, double x_offset, double y_offset);

    /**
     * Calibrates the IMU and tares all devices
     */
    void calibrate();

    // Taring (resetting) methods for the pose estimation.
    void tare(double x, double y, double yaw);
    void tare();

    // Returns true if the system is moving
    bool is_moving(double rads_s_threshold=0.02);
    
    /**
     * Retrieves the corrected and computed pose.
     * @return The current pose of the system.
     */
    TwoDPose get_pose();

    /**
     * Retrieves the previous pose from a last step
     */
    TwoDPose get_last_pose();
public: // This is one of the ONLY exceptions to be public, as another module requires this step function.
    /**
     * Override of ComputeNode's __step method to update the drivetrain's operation each cycle.
     */
    void __step_down(); // This steps down to the Odom Unit (meant to be managed by drivetrain or fusion object)
    void __step() override;  // Protected helper function for processing steps
};


#endif // WHOOP_DRIVETRAIN_HPP

