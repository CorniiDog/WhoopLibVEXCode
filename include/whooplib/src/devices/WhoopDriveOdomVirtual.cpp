/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDriveOdomVirtual.cpp                                 */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Odometry Estimation                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "whooplib/include/devices/WhoopDriveOdomVirtual.hpp"



/**
 * Constructor for Virtual Drive Odom.
 * The odom unit center is the virtual intercept of the perpendicular faces of the odometry trackers.
 * Visual Representation of Odom Location: https://imgur.com/x8ObCIG
 * @param odom_offset A pointer to the odometry offset
 */
WhoopDriveOdomVirtual::WhoopDriveOdomVirtual(WhoopDriveOdomOffset* odom_offset){
    this->odom_offset = odom_offset;
}

/**
 * Calibrates the IMU and tares all devices
 */
void WhoopDriveOdomVirtual::calibrate(){
    thread_lock.lock();
    odom_offset->calibrate();
    thread_lock.unlock();
    hard_tare();
}

void WhoopDriveOdomVirtual::_transform_pose(){
    // Also consider robot transformation
    TwoDPose transposed = tared_pose.toObjectSpace(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw);

    thread_lock.lock();
    this->pose.x = transposed.x + tare_x;
    this->pose.y = transposed.y + tare_y;
    this->pose.yaw = transposed.yaw;
    thread_lock.unlock();
}

// Taring (resetting) methods for the pose estimation.
void WhoopDriveOdomVirtual::tare(double x, double y, double yaw){
    thread_lock.lock();
    this->tare_x = x;
    this->tare_y = y;
    this->tare_yaw = yaw;

    tared_pose = TwoDPose(this->raw_pose.x, this->raw_pose.y, this->raw_pose.yaw - tare_yaw);
    thread_lock.unlock();
}

void WhoopDriveOdomVirtual::tare(){
    tare(0,0,0);
}

void WhoopDriveOdomVirtual::hard_tare(double x, double y, double yaw){
    thread_lock.lock();
    tared_pose = TwoDPose(0,0,0);
    odom_offset->tare(x, y, yaw);
    thread_lock.unlock();
}

void WhoopDriveOdomVirtual::hard_tare(){
    hard_tare(0,0,0);
}

// Returns true if the system is moving
bool WhoopDriveOdomVirtual::is_moving(double rads_s_threshold){
    return odom_offset->is_moving(rads_s_threshold);
}

/**
 * Retrieves the corrected and computed pose.
 * @return The current pose of the system.
 */
TwoDPose WhoopDriveOdomVirtual::get_pose(){
    thread_lock.lock();
    TwoDPose p = pose;
    thread_lock.unlock();
    return p;
}

void WhoopDriveOdomVirtual::__step(){
    thread_lock.lock();
    odom_offset->__step_down();
    raw_pose = odom_offset->get_pose();
    thread_lock.unlock();
    _transform_pose();
    
}
