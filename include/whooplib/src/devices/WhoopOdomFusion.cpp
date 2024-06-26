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

WhoopOdomFusion::WhoopOdomFusion(WhoopVision* whoop_vision, WhoopDriveOdomOffset* odom_offset, double min_confidence_threshold, FusionMode fusion_mode, double max_fusion_shift_meters, double max_fusion_shift_radians, double feedforward_gain){
    this->max_fusion_shift_meters = max_fusion_shift_meters/55.6;
    this->max_fusion_shift_radians = max_fusion_shift_radians/55.6;
    this->fusion_mode = fusion_mode;
    this->whoop_vision = whoop_vision;
    this->odom_offset = odom_offset;
    this->feedforward_gain = feedforward_gain/10.0;
    this->whoop_vision->on_update(std::bind(&WhoopOdomFusion::on_vision_pose_received, this, std::placeholders::_1));
}

void WhoopOdomFusion::on_vision_pose_received(Pose p){
    if(fusion_mode == FusionMode::wheel_odom_only){
        return;
    }
    
    self_lock.lock();
    if (p.confidence >= min_confidence_threshold) {
        double feedforward_x_delta = feedforward_gain * (pose.x - last_pose.x);
        double feedforward_y_delta = feedforward_gain * (pose.y - last_pose.y);
        double feedforward_yaw_delta = feedforward_gain * (pose.yaw - last_pose.yaw);

        double distance = std::sqrt(std::pow(p.x - pose.x, 2) + std::pow(p.y - pose.y, 2));
        double angle_difference = std::fabs(p.yaw - pose.yaw);

        // Handle linear position adjustment
        if (fusion_mode == FusionMode::fusion_gradual && distance > max_fusion_shift_meters) {
            // Calculate the direction vector from current to target position
            double dx = p.x - pose.x;
            double dy = p.y - pose.y;
            double norm = std::sqrt(dx * dx + dy * dy);

            // Scale the vector to the maximum allowable length
            dx = (dx / norm) * max_fusion_shift_meters;
            dy = (dy / norm) * max_fusion_shift_meters;

            // Update the pose by the maximum allowable shift
            pose.x += dx;
            pose.y += dy;
        } else {
            // If within allowable distance, update position to target
            pose.x = p.x;
            pose.y = p.y;
        }
        pose.x += feedforward_x_delta;
        pose.y += feedforward_y_delta;

        // Handle angular position adjustment
        if (fusion_mode == FusionMode::fusion_gradual && angle_difference > max_fusion_shift_radians) {
            // Adjust yaw by the maximum allowable radians directly toward the target yaw
            double dyaw = p.yaw - pose.yaw;
            pose.yaw += std::copysign(max_fusion_shift_radians, dyaw);
        } else {
            // If within allowable angle, update yaw to target
            pose.yaw = p.yaw;
        }
        pose.yaw += feedforward_yaw_delta;

        // Tare the odometer to the newly adjusted pose
        odom_offset->tare(pose.x, pose.y, pose.yaw);
    }
    pose.z = p.z; // Adjust Z
    pose.confidence = p.confidence; // Adjusts confidence

    last_pose = pose; // Store last p

    self_lock.unlock();
}

void WhoopOdomFusion::tare(double x, double y, double z, double yaw){
    self_lock.lock();
    whoop_vision->tare(x, y, z, 0, yaw, 0);
    odom_offset->tare(x, y, yaw);
    pose.x = x;
    pose.y = y;
    pose.z = z;
    pose.yaw = yaw;
    last_pose = pose;
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
    odom_offset->calibrate();
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
    return odom_offset->is_moving(rads_s_threshold);
}

void WhoopOdomFusion::__step(){
    self_lock.lock();

    if(fusion_mode != FusionMode::vision_only){
        odom_offset->__step_down(); // Step down wheel odometry ladder
        TwoDPose result = odom_offset->get_pose();
        pose.x = result.x;
        pose.y = result.y;
        pose.yaw = result.yaw;
    }
    pose.roll = odom_offset->odom_unit->inertial_sensor->get_roll_radians();
    pose.pitch = odom_offset->odom_unit->inertial_sensor->get_pitch_radians();
    self_lock.unlock();
}