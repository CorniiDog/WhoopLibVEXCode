/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopOdomCommunicator.cpp                                 */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Communicates Wheel Odometry to Jetson Nano                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopOdomCommunicator.hpp"


WhoopOdomCommunicator::WhoopOdomCommunicator(BufferNode* bufferSystem, RobotVisionOffset* vision_offset, WhoopDriveOdomOffset* odom_offset, std::string odom_stream, int pose_precision, int rolling_average_n):
    rolling_average_x(rolling_average_n),
    rolling_average_y(rolling_average_n),
    rolling_average_yaw(rolling_average_n){
    odom_messenger = std::make_unique<Messenger>(bufferSystem, odom_stream, deleteAfterRead::no_delete);
    this->odom_offset = odom_offset;
    this->pose_precision = pose_precision;
    this->vision_offset = vision_offset;
}


void WhoopOdomCommunicator::__step(){

    velocityVector vel_vector = odom_offset->get_velocity_vector(TwoDPose(vision_offset->x, vision_offset->y, 0));

    if(!vel_vector.is_clean){ // If dirty (tared)
        return;
    }

    TwoDPose vel_vector_2d = TwoDPose(vel_vector.x, vel_vector.y, vel_vector.yaw);

    vel_vector_2d.x = rolling_average_x.process(vel_vector_2d.x);
    vel_vector_2d.y = rolling_average_y.process(vel_vector_2d.y);
    vel_vector_2d.yaw = rolling_average_yaw.process(vel_vector_2d.yaw);

    relative_velocity = vel_vector_2d; // Update pose deltas for robot

    odom_messenger->send(vel_vector_2d.to_realsense_string(pose_precision));
}