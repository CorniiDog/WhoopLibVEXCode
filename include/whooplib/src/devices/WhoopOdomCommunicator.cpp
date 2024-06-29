/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopOdomCommunicator.cpp                                 */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Communicates Wheel Odometry to Jetson Nano                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopOdomCommunicator.hpp"


WhoopOdomCommunicator::WhoopOdomCommunicator(BufferNode* bufferSystem, RobotVisionOffset* vision_offset, WhoopDriveOdomOffset* odom_offset, std::string odom_stream, int pose_precision){
    odom_messenger = std::make_unique<Messenger>(bufferSystem, odom_stream, deleteAfterRead::no_delete);
    this->odom_offset = odom_offset;
    this->pose_precision = pose_precision;
    this->vision_offset = vision_offset;
}


void WhoopOdomCommunicator::__step(){
    TwoDPose pose = odom_offset->get_pose(); // get pose
    TwoDPose last_pose = odom_offset->get_last_pose(); // get last pose

    pose *= TwoDPose(vision_offset->x, vision_offset->y, 0); // Apply offset to position of realsense device
    last_pose *= TwoDPose(vision_offset->x, vision_offset->y, 0); // Apply offset to position of realsense device

    TwoDPose pose_deltas = last_pose.toObjectSpace(pose);
    pose_deltas.x /= 0.01; // Convert to meters/second
    pose_deltas.y /= 0.01; // Convert to meters/second
    pose_deltas.yaw /= 0.01; // Convert to radians/second

    relative_velocity = pose_deltas; // Update pose deltas for robot

    odom_messenger->send(pose_deltas.to_realsense_string(pose_precision));
}