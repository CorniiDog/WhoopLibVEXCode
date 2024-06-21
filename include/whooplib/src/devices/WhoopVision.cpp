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


WhoopVision::WhoopVision(BufferNode* bufferSystem, std::string pose_stream): tared_position(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw - tare_yaw){
    setup_messenger(bufferSystem, pose_stream);
}

void WhoopVision::_transform_pose(){
    TwoDPose transposed = tared_position.toObjectSpace(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw);

    thread_lock.lock();
    this->pose.x = transposed.x + tare_x;
    this->pose.y = transposed.y + tare_y;
    this->pose.z = this->raw_pose.z - tared_z;
    this->pose.pitch = this->raw_pose.pitch - tared_pitch;
    this->pose.yaw = transposed.yaw;
    this->pose.roll = this->raw_pose.roll - tared_roll;
    thread_lock.unlock();
}

void WhoopVision::tare(double x, double y, double z, double pitch, double yaw, double roll){
    thread_lock.lock();
    this->tare_x = x;
    this->tare_y = y;
    this->tare_z = z;
    this->tare_pitch = pitch;
    this->tare_yaw = yaw;
    this->tare_roll = roll;

    tared_z = this->raw_pose.z - tare_z;
    tared_pitch = this->raw_pose.pitch - tare_pitch;
    tared_roll = this->raw_pose.roll - tare_roll;

    TwoDPose tared_p(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw - tare_yaw);
    this->tared_position = tared_p;
    thread_lock.unlock();
}

void WhoopVision::tare(double x, double y, double yaw, tare_rest tare_rest_to_zero){
    thread_lock.lock();
    this->tare_x = x;
    this->tare_y = y;
    this->tare_yaw = yaw;

    if (tare_rest_to_zero == tare_rest::do_tare){
        this->tare_z = 0;
        this->tare_pitch = 0;
        this->tare_roll = 0;

        tared_z = this->raw_pose.z - tare_z;
        tared_pitch = this->raw_pose.pitch - tare_pitch;
        tared_roll = this->raw_pose.roll - tare_roll;
    }

    TwoDPose tared_p(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw - tare_yaw);
    this->tared_position = tared_p;
    thread_lock.unlock();
}

void WhoopVision::tare(double x, double y, double yaw){
    tare(x, y, yaw, tare_rest::do_tare);
}

void WhoopVision::_update_pose(std::string pose_data){
    // Note: Data retrieved from Jetson Nano is using Graphics Coordinate System (assuming rotation is 0,0,0 for standardization):
    // In Graphics Coordinate System for Realsense: +X is right, -Z is going forwards, +Y is up
    // We correct this to follow robotics coordinate system: +X is right, +Y is forwards, +Z is up
    // Both are pitch, yaw, roll equivalent.
    std::istringstream iss(pose_data); // Create a string stream from the input string
    double negative_y;
    thread_lock.lock();
    iss >> raw_pose.x >> raw_pose.z >> negative_y >> raw_pose.pitch >> raw_pose.yaw >> raw_pose.roll;
    raw_pose.y = -negative_y;
    thread_lock.unlock();

    this->_transform_pose();
}


Pose WhoopVision::get_pose(){
    thread_lock.lock();
    Pose p = pose;
    thread_lock.unlock();
    return p;
}