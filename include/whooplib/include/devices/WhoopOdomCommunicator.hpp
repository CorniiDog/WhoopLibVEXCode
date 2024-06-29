/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopOdomCommunicator.hpp                                 */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Communicates Wheel Odometry to Jetson Nano                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_ODOM_COMMUNICATOR_HPP
#define WHOOP_ODOM_COMMUNICATOR_HPP

#include "whooplib/include/devices/WhoopDriveOdomOffset.hpp"

/**
 * Class responsible for communicating odometry
 */
class WhoopOdomCommunicator : public ComputeNode {

protected:
    // Upon initialization
    std::unique_ptr<Messenger> odom_messenger = nullptr; // Handles messaging for pose data from Jetson Nano

public:
    WhoopDriveOdomOffset* odom_offset;
    int pose_precision;
    RobotVisionOffset* vision_offset;
    /**
     * This constructs an odometry communicator for the drivetrain so that it can send for the T265 to parse
     * @param bufferSystem Pointer to the buffer system to communicate the messenger to
     * @param odom_offset Pointer to the drive offset object
     * @param odom_stream The string that represents the odometry stream to send over
     * @param pose_precision The number of decimal places of the pose data (measurements in meters/radians). Higher decimal places is better precision, but larger serial packets
     */
    WhoopOdomCommunicator(BufferNode* bufferSystem, RobotVisionOffset* vision_offset, WhoopDriveOdomOffset* odom_offset, std::string odom_stream, int pose_precision);
   
public: // This is one of the ONLY exceptions to be public, as another module requires this step function.
    /**
     * Override of ComputeNode's __step method to update the drivetrain's operation each cycle.
     */
    void __step() override;  // Protected helper function for processing steps
};


#endif