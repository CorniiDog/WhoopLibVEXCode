/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopVision.cpp                                           */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A Streamlined Jetson Nano Vision System                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "whooplib/include/devices/WhoopVision.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <memory> // For std::unique_ptr

RobotVisionOffset::RobotVisionOffset(double x, double y)
{
    this->x = x;
    this->y = y;
}

void WhoopVision::setup_messenger(BufferNode *bufferSystem, const std::string &pose_stream)
{
    pose_messenger = std::make_unique<Messenger>(bufferSystem, pose_stream, deleteAfterRead::no_delete);
    pose_messenger->on_message(std::bind(&WhoopVision::_update_pose, this, std::placeholders::_1));
}

WhoopVision::WhoopVision(RobotVisionOffset *robotOffset, BufferNode *bufferSystem, std::string pose_stream) : tared_position(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw - tare_yaw), offset_change(0, 0, 0)
{
    robot_offset = robotOffset;
    setup_messenger(bufferSystem, pose_stream);
    this->tare();
}

void WhoopVision::_transform_pose(bool apply_delta)
{
    // Also consider robot transformation
    TwoDPose transformed = tared_position.toObjectSpace(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw);

    // Ensure robot offset is correctly applied
    TwoDPose offset(-robot_offset->x, -robot_offset->y, 0);

    // Acquire relative delta change of robot relative to vision system if tare
    if (apply_delta)
    {
        this->offset_change = transformed.toWorldSpace(-offset);
    }

    // Apply robot offset to transformation
    transformed *= offset;

    thread_lock.lock();
    this->pose.x = transformed.x + tare_x + this->offset_change.x;
    this->pose.y = transformed.y + tare_y + this->offset_change.y;
    this->pose.z = this->raw_pose.z - tared_z;
    this->pose.pitch = this->raw_pose.pitch - tared_pitch;
    this->pose.yaw = transformed.yaw;
    this->pose.roll = this->raw_pose.roll - tared_roll;
    this->pose.confidence = confidence;
    thread_lock.unlock();
}

void WhoopVision::tare(double x, double y, double z, double pitch, double yaw, double roll)
{
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

    this->tared_position = TwoDPose(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw - tare_yaw);
    this->_transform_pose(true);
    thread_lock.unlock();
}

void WhoopVision::on_update(std::function<void(Pose)> callback)
{
    callback_functions.push_back(callback);
}

void WhoopVision::tare(double x, double y, double yaw, tare_remaining_0 tare_rest_to_zero)
{
    thread_lock.lock();
    this->tare_x = x;
    this->tare_y = y;
    this->tare_yaw = yaw;

    if (tare_rest_to_zero == tare_remaining_0::do_tare)
    {
        this->tare_z = 0;
        this->tare_pitch = 0;
        this->tare_roll = 0;

        tared_z = this->raw_pose.z - tare_z;
        tared_pitch = this->raw_pose.pitch - tare_pitch;
        tared_roll = this->raw_pose.roll - tare_roll;
    }

    TwoDPose tared_p(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw - tare_yaw);
    this->tared_position = tared_p;
    this->_transform_pose(true);
    thread_lock.unlock();
}

void WhoopVision::tare(double x, double y, double yaw)
{
    tare(x, y, yaw, tare_remaining_0::do_tare);
}

void WhoopVision::tare()
{
    tare(0, 0, 0, 0, 0, 0);
}

void WhoopVision::_update_pose(std::string pose_data)
{
    // Note: Data retrieved from Jetson Nano is using Graphics Coordinate System (assuming rotation is 0,0,0 for standardization):
    // In Graphics Coordinate System for Realsense: +X is right, -Z is going forwards, +Y is up
    // We correct this to follow robotics coordinate system: +X is right, +Y is forwards, +Z is up
    // Both are pitch, yaw, roll equivalent.
    std::istringstream iss(pose_data); // Create a string stream from the input string

    double negative_x, y, z, pitch, yaw, roll, unscaled_confidence;
    if (!(iss >> negative_x >> z >> y >> pitch >> yaw >> roll >> unscaled_confidence))
    {
        return; // Reject malformed data
    }

    last_vision_message_time = Brain.Timer.time(msec);

    thread_lock.lock();
    confidence = unscaled_confidence / 3.0; // Scale from 0 to 1
    raw_pose.x = -negative_x;
    raw_pose.y = y;
    raw_pose.z = z;
    raw_pose.pitch = pitch;
    raw_pose.yaw = yaw;
    raw_pose.roll = roll;
    raw_pose.confidence = confidence;
    thread_lock.unlock();

    for (size_t i = 0; i < callback_functions.size(); ++i)
    {
        callback_functions[i](pose);
    }

    this->_transform_pose();
}

bool WhoopVision::vision_running()
{
    return (Brain.Timer.time(msec) - last_vision_message_time) < 500;
}

Pose WhoopVision::get_pose()
{
    thread_lock.lock();
    Pose p = pose;
    thread_lock.unlock();
    return p;
}