/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       TwoDPose.cpp                                              */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Two-Dimensional CFrames                                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/TwoDPose.hpp"
#include <cmath>
#include "whooplib/include/toolbox.hpp"

TwoDPose::TwoDPose(double x, double y, double yaw) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
}

TwoDPose TwoDPose::multiplicative_delta(const TwoDPose& other) const{
    // Calculate the new position
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    double delta_x = other.x * cos_yaw - other.y * sin_yaw;
    double delta_y = other.x * sin_yaw + other.y * cos_yaw;
    double delta_yaw = other.yaw;

    return TwoDPose(delta_x, delta_y, normalize_angle(delta_yaw));
}

TwoDPose TwoDPose::operator*(const TwoDPose& other) const {
    // Calculate the new position
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    double new_x = x + other.x * cos_yaw - other.y * sin_yaw;
    double new_y = y + other.x * sin_yaw + other.y * cos_yaw;

    // Calculate the new yaw
    double new_yaw = yaw + other.yaw;

    return TwoDPose(new_x, new_y, normalize_angle(new_yaw));
}

TwoDPose& TwoDPose::operator*=(const TwoDPose& other) {
    *this = *this * other;
    return *this;
}

TwoDPose TwoDPose::toObjectSpace(const TwoDPose& other) const {
    return this->toObjectSpace(other.x, other.y, other.yaw);
}

TwoDPose TwoDPose::toObjectSpace(double x, double y, double yaw) const {
    // Calculate the relative position
    double dx = x - this->x;
    double dy = y - this->y;

    // Create the rotation matrix for -yaw
    double cos_yaw = cos(-this->yaw);
    double sin_yaw = sin(-this->yaw);
    
    // Apply the rotation matrix
    double relative_x = dx * cos_yaw - dy * sin_yaw;
    double relative_y = dx * sin_yaw + dy * cos_yaw;

    // Calculate the relative yaw
    double relative_yaw = yaw - this->yaw;

    return TwoDPose(relative_x, relative_y, normalize_angle(relative_yaw));
}

TwoDPose TwoDPose::inverse() const {
    // Calculate the inverse rotation and translation
    double cos_yaw = cos(-this->yaw); // Inverse rotation is simply the negative of the yaw
    double sin_yaw = sin(-this->yaw);
    
    // Apply the inverse rotation to -x, -y
    double inv_x = -this->x * cos_yaw - -this->y * sin_yaw;
    double inv_y = -this->x * sin_yaw + -this->y * cos_yaw;

    return TwoDPose(inv_x, inv_y, -this->yaw);
}

TwoDPose TwoDPose::inverseMultiply(const TwoDPose& other) const {
    TwoDPose inv = this->inverse();
    return inv * other;
}

TwoDPose TwoDPose::operator/(const TwoDPose& other) const{
    return inverseMultiply(other);
}

TwoDPose& TwoDPose::operator/=(const TwoDPose& other){
    *this = *this / other;
    return *this;
}


TwoDPose TwoDPose::toWorldSpace(const TwoDPose& other) const {
    // Apply rotation to the point using this object's yaw
    double cos_yaw = cos(this->yaw);
    double sin_yaw = sin(this->yaw);

    double global_x = this->x + other.x * cos_yaw - other.y * sin_yaw;
    double global_y = this->y + other.x * sin_yaw + other.y * cos_yaw;
    double global_yaw = normalize_angle(this->yaw + other.yaw);

    return TwoDPose(global_x, global_y, global_yaw);
}
