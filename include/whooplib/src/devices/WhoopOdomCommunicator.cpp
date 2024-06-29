/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopOdomCommunicator.cpp                                 */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Communicates Wheel Odometry to Jetson Nano                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopOdomCommunicator.hpp"


WhoopOdomCommunicator::WhoopOdomCommunicator(BufferNode* bufferSystem, WhoopDriveOdomOffset* drive_offset, std::string odom_stream, int pose_precision){
    odom_messenger = std::make_unique<Messenger>(bufferSystem, odom_stream, deleteAfterRead::no_delete);
    this->drive_offset = drive_offset;
    this->pose_precision = pose_precision;
}


void WhoopOdomCommunicator::__step(){
    TwoDPose pose = drive_offset->get_pose();
    odom_messenger->send(pose.to_string(pose_precision));
}