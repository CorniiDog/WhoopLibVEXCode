/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       TwoDPose.cpp                                              */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Two-Dimensional CFrames                                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/TwoDPose.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <string>
#include "whooplib/include/toolbox.hpp"

TwoDPose::TwoDPose(double x, double y, double yaw) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
}

TwoDPose TwoDPose::multiplicative_delta(const TwoDPose& other) const{
    TwoDPose p = toWorldSpace(other);
    p.yaw = yaw;
    return p;
}

TwoDPose TwoDPose::operator*(const TwoDPose& other) const {
    return toWorldSpace(other);
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

    double cos_yaw = cos(this->yaw);
    double sin_yaw = sin(this->yaw);
    
    // Apply the rotation matrix
    double relative_x = dx * cos_yaw - dy * sin_yaw;
    double relative_y = dx * sin_yaw + dy * cos_yaw;

    // Calculate the relative yaw
    double relative_yaw = yaw - this->yaw;

    return TwoDPose(relative_x, relative_y, normalize_angle(relative_yaw));
}

TwoDPose TwoDPose::operator-() const {
    return TwoDPose(-x, -y, -yaw);
}

TwoDPose TwoDPose::inverse() const {
    // Calculate the inverse rotation
    double cos_yaw = cos(this->yaw);
    double sin_yaw = sin(this->yaw); 
    double cos_yaw_flipped = cos(-this->yaw);
    double sin_yaw_flipped = sin(-this->yaw);
    
    // Apply the inverse rotation to -x, -y
    double inv_x = this->y * sin_yaw - this->x * cos_yaw; // Rotate the negated translation
    double inv_y = this->x * sin_yaw_flipped - this->y * cos_yaw_flipped;  // components

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
    double cos_yaw = cos(other.yaw - this->yaw);
    double sin_yaw = sin(other.yaw - this->yaw);

    double global_x = this->x + other.x * sin_yaw + other.y * cos_yaw;
    double global_y = this->y + other.y * sin_yaw - other.x * cos_yaw;
    double global_yaw = normalize_angle(other.yaw - this->yaw);

    return TwoDPose(global_x, global_y, global_yaw);
}


std::string TwoDPose::to_string(int decimal_places) {
    std::ostringstream oss;
    if (decimal_places >= 0) {
        oss << std::fixed << std::setprecision(decimal_places);
    }
    oss << x << " " << y << " " << yaw;
    return oss.str();
}

std::string TwoDPose::to_realsense_string(int decimal_places) {
    std::ostringstream oss;
    if (decimal_places >= 0) {
        oss << std::fixed << std::setprecision(decimal_places);
    }
    oss << x << " " << -y << " " << yaw;
    return oss.str();
}

