/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopOdomFusion.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 25 2024                                           */
/*    Description:  Fuses Wheel and Visual Odometry                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_ODOM_FUSION_HPP
#define WHOOP_ODOM_FUSION_HPP

#include "whooplib/include/devices/WhoopDriveOdomUnit.hpp"
#include "whooplib/include/devices/WhoopDriveOdomOffset.hpp"
#include "whooplib/include/devices/WhoopVision.hpp"
#include "whooplib/include/toolbox.hpp"
#include "vex.h"
#include <vector>
#include <memory>

// Enumeration defining possible fusion modes between visual and wheel odometry data.
enum FusionMode {
    fusion_instant,  // Instantly aligns wheel odometry to vision odometry upon data retrieval.
    fusion_gradual,  // Gradually aligns wheel odometry to vision odometry over time.
    vision_only,     // Vision Odometry Only
    wheel_odom_only  // Wheel Odometry Only
};

// Class responsible for fusing visual and wheel odometry data.
class WhoopOdomFusion : public ComputeNode {
protected:
    vex::mutex self_lock;  // Mutex for thread-safe operations.
    WhoopDriveOdomOffset* odom_offset;  // Pointer to the wheel odometry offset object.
    WhoopVision* whoop_vision;  // Pointer to the vision odometry unit.
    double min_confidence_threshold;  // Minimum confidence level required to accept new vision data.
    FusionMode fusion_mode;  // Current mode of odometry data fusion.
    double max_fusion_shift_meters;  // Maximum shift in meters per step when gradually fusing data.
    double max_fusion_shift_radians;  // Maximum rotational shift in radians per step when gradually fusing data.
    double feedforward_gain; // Feedforward gain of the vision system as it has delay

    // Callback function that handles new vision pose data.
    void on_vision_pose_received(Pose p);

    Pose last_pose = Pose();
public:
    Pose pose = Pose();  // Current fused pose of the odometry system.

    /**
     * Constructs a new odometry fusion object.
     * @param whoop_vision Pointer to the vision odometry system.
     * @param odom_offset Pointer to the wheel odometry offset object.
     * @param min_confidence_threshold Minimum confidence required to consider vision data (0.0 - 1.0).
     * @param fusion_mode Method of fusing vision with wheel odometry (instant, gradual, vision_only, wheel_odom_only).
     * @param max_fusion_shift_meters If FusionMode is fusion_gradual, it is the maximum allowable shift in meters for gradual fusion, per second.
     * @param max_fusion_shift_radians If FusionMode is fusion_gradual, it is the maximum allowable rotational shift in radians for gradual fusion, per second.
     * @param feedforward_gain // Feedforward gain of the vision system as it has delay, in milliseconds (For 100Hz Wheel Odometry)
     */
    WhoopOdomFusion(WhoopVision* whoop_vision, WhoopDriveOdomOffset* odom_offset, double min_confidence_threshold, FusionMode fusion_mode, double max_fusion_shift_meters, double max_fusion_shift_radians, double feedforward_gain);

    /**
     * Retreives the pose from the odom fusion
     * @returns Pose object
     */
    Pose get_pose();

    /**
     * Runs calibration process
     */
    void calibrate();

    /**
     * Sets the current odometry to the specified coordinates and yaw.
     * @param x the x coordinate (forwards), in meters
     * @param y the y coordinate (right), in meters
     * @param yaw the yaw (counter-clockwise), in radians
     */
    void tare(double x, double y, double yaw);

    /**
     * Sets the current odometry to the specified coordinates in 3D space, and yaw.
     * @param x the x coordinate (forwards), in meters
     * @param y the y coordinate (right), in meters
     * @param z the z coordinate (up), in meters
     * @param yaw the yaw (counter-clockwise), in radians
     */
    void tare(double x, double y, double z, double yaw);
    
    // Resets the current odometry to the origin (0,0,0).
    void tare();

public:  // Exceptionally public to allow access from another module.
    /**
     * Processes a single step of odometry updates.
     * This method is called periodically to integrate new sensor data and adjust the internal state.
     */
    void __step() override;  // Protected helper function for processing steps
};

#endif // WHOOP_ODOM_FUSION_HPP
