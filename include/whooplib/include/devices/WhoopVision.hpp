/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopVision.hpp                                           */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A Streamlined Jetson Nano Vision System                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_VISION_HPP
#define WHOOP_VISION_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/calculators/TwoDPose.hpp"
#include "vex.h"
#include <vector>
#include <memory>

/**
 * Struct representing a three-dimensional pose with orientation.
 */
struct Pose{
    double x=0, y=0, z=0;
    double pitch=0, yaw=0, roll=0;
};

/**
 * Enum to control whether taring (resetting) operations should be applied.
 */
enum tare_remaining_0{
    do_tare=true,
    dont_tare=false
};

/**
 * Represents an offset used for vision-based calculations.
 */
class RobotVisionOffset{
public:
    double x = 0;
    double y = 0;

    /**
     * Constructor to set the x and y offsets.
     * @param x Horizontal offset.
     * @param y Vertical offset.
     */
    RobotVisionOffset(double x, double y); 
};

/**
 * Manages vision processing for robotics, handling pose estimation and transformations based on vision sensor input.
 */
class WhoopVision {
protected:
    // Upon initialization
    std::unique_ptr<Messenger> pose_messenger = nullptr; // Handles messaging for pose data from Jetson Nano
    Pose raw_pose; // Raw pose data from vision sensor.

    // Tares
    double tare_x = 0;
    double tare_y = 0;
    double tare_z = 0;
    double tare_pitch = 0;
    double tare_roll = 0;
    double tare_yaw = 0;

    // Tared computes
    double tared_z = this->raw_pose.z - tare_z;
    double tared_pitch = this->raw_pose.pitch - tare_pitch;
    double tared_roll = this->raw_pose.roll - tare_roll;
    TwoDPose tared_position; // Position adjusted for tare.
    TwoDPose offset_change; // Computed change due to offset adjustments.

    RobotVisionOffset* robot_offset; // Offset configuration for vision adjustments.

    /**
     * Setups the messaging system for receiving pose data.
     * @param bufferSystem The buffer node system used for message handling.
     * @param pose_stream The stream identifier for pose data.
     */
    void setup_messenger(BufferNode* bufferSystem, const std::string& pose_stream);

    /**
     * Transforms the raw pose data based on the current configuration and tare settings.
     * @param apply_delta Flag to determine if delta adjustments should be applied.
     */
    void _transform_pose(bool apply_delta=false);

    /**
     * Updates the pose based on incoming data.
     * @param pose_data The string containing serialized pose data.
     */
    void _update_pose(std::string pose_data);
public:
    vex::mutex thread_lock; // Mutex for synchronization of pose data updates.

    Pose pose; // The corrected and computed pose of the robot.

    /**
     * Constructor for initializing the vision system with a specific configuration.
     * @param robotOffset The offset configuration for vision calculations.
     * @param bufferSystem The buffer node system for data handling.
     * @param pose_stream The stream identifier for incoming pose data.
     */
    WhoopVision(RobotVisionOffset* robotOffset, BufferNode* bufferSystem, std::string pose_stream); 

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
};


#endif // WHOOP_VISION_HPP

