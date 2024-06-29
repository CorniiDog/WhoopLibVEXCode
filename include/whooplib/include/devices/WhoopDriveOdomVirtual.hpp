/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDriveOdomVirtual.hpp                                 */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Odometry Estimation                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_DRIVE_ODOM_VIRTUAL_HPP
#define WHOOP_DRIVE_ODOM_VIRTUAL_HPP

#include "whooplib/include/devices/WhoopDriveOdomOffset.hpp"

/**
 * Class responsible for virtually deriving Odometry data.
 */
class WhoopDriveOdomVirtual : public ComputeNode {

private:
    TwoDPose tared_pose = TwoDPose(0,0,0); // Tared pose for utilization
    TwoDPose raw_pose = TwoDPose(0,0,0);

    double tare_x = 0;
    double tare_y = 0;
    double tare_yaw = 0;

    void _transform_pose();

public:
    WhoopDriveOdomOffset* odom_offset;

    TwoDPose pose = TwoDPose(0,0,0);
    vex::mutex thread_lock;  // Mutex for synchronizing access to odometry components.

    /**
     * Constructor for Virtual Drive Odom.
     * The odom unit center is the virtual intercept of the perpendicular faces of the odometry trackers.
     * Visual Representation of Odom Location: https://imgur.com/x8ObCIG
     * @param odom_offset A pointer to the odometry offset
     */
    WhoopDriveOdomVirtual(WhoopDriveOdomOffset* odom_offset);

    /**
     * Calibrates the IMU and tares all devices
     */
    void calibrate();

    // Taring (resetting) methods for the pose estimation.
    void tare(double x, double y, double yaw);
    void tare();

    // Hard tares. This means it steps down the latter instead of being a virtual tare.
    void hard_tare(double x, double y, double yaw);
    void hard_tare();

    // Returns true if the system is moving
    bool is_moving(double rads_s_threshold=0.02);
    
    /**
     * Retrieves the corrected and computed pose.
     * @return The current pose of the system.
     */
    TwoDPose get_pose();
public: // This is one of the ONLY exceptions to be public, as another module requires this step function.
    /**
     * Override of ComputeNode's __step method to update the drivetrain's operation each cycle.
     */
    void __step() override;  // Protected helper function for processing steps
};


#endif