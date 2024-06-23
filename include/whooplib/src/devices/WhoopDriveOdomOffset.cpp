/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDriveOdomOffset.cpp                                  */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Odometry Offset Module for Pose Estimation                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/calculators/TwoDPose.hpp"
#include "whooplib/include/devices/WhoopDriveOdomOffset.hpp"
#include "vex.h"
#include <vector>
#include <memory>

WhoopDriveOdomOffset::WhoopDriveOdomOffset(WhoopDriveOdomUnit* odom_unit, double x_offset, double y_offset):
    offset(x_offset, y_offset, 0){
    this->odom_unit = odom_unit;
}


void WhoopDriveOdomOffset::calibrate(){
    thread_lock.lock();
    odom_unit->calibrate();
    thread_lock.unlock();
    tare();
}


void WhoopDriveOdomOffset::tare(double x, double y, double yaw){
    thread_lock.lock();
    TwoDPose TaredOffset = TwoDPose(x, y, yaw)/offset;
    odom_unit->tare(TaredOffset.x, TaredOffset.y, TaredOffset.yaw);
    thread_lock.unlock();
}

void WhoopDriveOdomOffset::tare(){
    tare(0,0,0);
}


TwoDPose WhoopDriveOdomOffset::get_pose(){
    return pose;
}

void WhoopDriveOdomOffset::__step(){
    thread_lock.lock();
    odom_unit->__step();

    pose = odom_unit->pose * offset; // Update pose with offset

    thread_lock.unlock();
}
