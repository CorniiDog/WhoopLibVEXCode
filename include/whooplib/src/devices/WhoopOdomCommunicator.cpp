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
    TwoDPose pose = odom_offset->get_pose();
    odom_messenger->send(pose.to_realsense_string(pose_precision) + " " + doubleToString(vision_offset->x) + " " + doubleToString(-vision_offset->y));
}