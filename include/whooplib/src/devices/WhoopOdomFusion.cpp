/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopOdomFusion.cpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 25 2024                                           */
/*    Description:  Fuses Wheel and Visual Odometry                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopOdomFusion.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>

WhoopOdomFusion::WhoopOdomFusion(WhoopVision* whoop_vision, WhoopDriveOdomOffset* odom_offset, double min_confidence_threshold, FusionMode fusion_mode, double max_fusion_shift_meters, double max_fusion_shift_radians):
    odom_virtual(WhoopDriveOdomVirtual(odom_offset)){
    this->max_fusion_shift_meters = max_fusion_shift_meters/55.6;
    this->max_fusion_shift_radians = max_fusion_shift_radians/55.6;
    this->fusion_mode = fusion_mode;
    this->whoop_vision = whoop_vision;
    this->whoop_vision->on_update(std::bind(&WhoopOdomFusion::on_vision_pose_received, this, std::placeholders::_1));
}

void WhoopOdomFusion::on_vision_pose_received(Pose p){
    if(fusion_mode == FusionMode::wheel_odom_only){
        return;
    }
    
    self_lock.lock();
    if (p.confidence >= min_confidence_threshold) {
        frame_rejected = false;
        // Normalize angle difference to handle angle wrapping correctly
        double yaw_difference = normalize_angle(p.yaw - pose.yaw);

        double distance = std::sqrt(std::pow(p.x - pose.x, 2) + std::pow(p.y - pose.y, 2));
        double angle_difference = std::fabs(yaw_difference);

        // Handle linear position adjustment
        if (fusion_mode == FusionMode::fusion_gradual && distance > max_fusion_shift_meters) {
            double dx = p.x - pose.x;
            double dy = p.y - pose.y;
            double norm = std::sqrt(dx * dx + dy * dy);

            dx = (dx / norm) * max_fusion_shift_meters;
            dy = (dy / norm) * max_fusion_shift_meters;

            pose.x += dx;
            pose.y += dy;
        } else {
            pose.x = p.x;
            pose.y = p.y;
        }

        // Handle angular position adjustment
        if (fusion_mode == FusionMode::fusion_gradual && angle_difference > max_fusion_shift_radians) {
            pose.yaw += std::copysign(max_fusion_shift_radians, yaw_difference);
        } else {
            pose.yaw = p.yaw;
        }
        pose.yaw = normalize_angle(pose.yaw);

        odom_virtual.tare(pose.x, pose.y, pose.yaw);
    }
    else{
        frame_rejected = true;
    }
    pose.z = p.z;
    pose.confidence = p.confidence;

    self_lock.unlock();
}

void WhoopOdomFusion::tare(double x, double y, double z, double yaw){
    self_lock.lock();
    whoop_vision->tare(x, y, z, 0, yaw, 0);

    // Hard tare to sync with the Vision System's position
    Pose raw_pose = whoop_vision->raw_pose;
    odom_virtual.hard_tare(raw_pose.x, raw_pose.y, raw_pose.yaw);

    // Soft tare to apply transform that applies to what we need
    odom_virtual.tare(x, y, yaw);
    pose.x = x;
    pose.y = y;
    pose.z = z;
    pose.yaw = yaw;
    self_lock.unlock();
}

void WhoopOdomFusion::tare(double x, double y, double yaw){
    tare(x, y, 0, yaw);
}

void WhoopOdomFusion::tare(){
    tare(0,0,0);
}

void WhoopOdomFusion::calibrate(){
    self_lock.lock();
    odom_virtual.calibrate();
    whoop_vision->tare();
    self_lock.unlock();
}

Pose WhoopOdomFusion::get_pose(){
    self_lock.lock();
    Pose p = pose;
    self_lock.unlock();
    return p;
}

bool WhoopOdomFusion::is_moving(double rads_s_threshold){
    return odom_virtual.is_moving(rads_s_threshold);
}

bool WhoopOdomFusion::approving_frames(){
    return !frame_rejected;
}

void WhoopOdomFusion::__step(){
    self_lock.lock();

    if(fusion_mode != FusionMode::vision_only){
        odom_virtual.__step(); // Step down wheel odometry ladder
        TwoDPose result = odom_virtual.get_pose();
        pose.x = result.x;
        pose.y = result.y;
        pose.yaw = result.yaw;
    }
    pose.roll = odom_virtual.odom_offset->odom_unit->inertial_sensor->get_roll_radians();
    pose.pitch = odom_virtual.odom_offset->odom_unit->inertial_sensor->get_pitch_radians();
    self_lock.unlock();
}