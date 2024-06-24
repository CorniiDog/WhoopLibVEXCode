/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDriveOdomOffset.cpp                                  */
/*    Author:       Connor White (WHOOP)                                      */
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
#include <iostream>

WhoopDriveOdomOffset::WhoopDriveOdomOffset(WhoopDriveOdomUnit* odom_unit, double x_offset, double y_offset):
    offset(-x_offset, -y_offset, 0){
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

    TwoDPose TaredOffset = TwoDPose(x, y, yaw);

    // If there is an offset
    if(offset.x != 0 or offset.y != 0 or offset.yaw != 0){ 
        TaredOffset *= -offset;
    }

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

    if(offset.x == offset.y == offset.yaw == 0){ // If offset is not applied
        pose = odom_unit->pose; // Update pose without offset, to reduce computational time
    }
    else{
        pose = odom_unit->pose * offset; // Update pose with offset
    }

    thread_lock.unlock();
}
