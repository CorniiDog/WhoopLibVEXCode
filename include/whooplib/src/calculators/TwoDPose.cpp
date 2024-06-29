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


TwoDPose TwoDPose::toWorldSpace(const TwoDPose& other) const {
    // Apply rotation to the point using this object's yaw
    double cos_yaw = cos(this->yaw);
    double sin_yaw = sin(this->yaw);

    //double global_x = this->x + other.x * cos_yaw - other.y * sin_yaw;
    //double global_y = this->y + other.x * sin_yaw + other.y * cos_yaw;

    double global_x = this->y + other.y * sin_yaw + other.x * cos_yaw;
    if(sin_yaw == 0){
        sin_yaw = 0.00000001;
    }
    double global_y = ((other.y * sin_yaw + other.x * cos_yaw) * cos_yaw - other.x + sin_yaw * this->y) / sin_yaw;

    double global_yaw = normalize_angle(this->yaw + other.yaw);

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