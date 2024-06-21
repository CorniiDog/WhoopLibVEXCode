#include "vex.h"
#include "whooplib/include/devices/WhoopVision.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <memory> // For std::unique_ptr

void WhoopVision::setup_messenger(BufferNode* bufferSystem, const std::string& pose_stream) {
    pose_messenger = std::make_unique<Messenger>(bufferSystem, pose_stream, deleteAfterRead::no_delete);
    pose_messenger->on_message(std::bind(&WhoopVision::_update_pose, this, std::placeholders::_1));
}


WhoopVision::WhoopVision(BufferNode* bufferSystem, std::string pose_stream){
    setup_messenger(bufferSystem, pose_stream);
}

void WhoopVision::_update_pose(std::string pose_data){
    // Note: Data retrieved from Jetson Nano is using Graphics Coordinate System (assuming rotation is 0,0,0 for standardization):
    // In Graphics Coordinate System for Realsense: +X is right, -Z is going forwards, +Y is up
    // We correct this to follow robotics coordinate system: +X is right, +Y is forwards, +Z is up
    // Both are pitch, yaw, roll equivalent.
    std::istringstream iss(pose_data); // Create a string stream from the input string
    double negative_y;
    thread_lock.lock();
    iss >> pose.x >> pose.z >> negative_y >> pose.pitch >> pose.yaw >> pose.roll;
    pose.y = -negative_y;
    thread_lock.unlock();
}


Pose WhoopVision::get_pose(){
    return pose;
}