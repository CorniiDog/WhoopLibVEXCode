
#include "whooplib/include/calculators/TwoDPose.hpp"
#include <cmath>
#include "whooplib/include/toolbox.hpp"

TwoDPose::TwoDPose(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}

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

TwoDPose& TwoDPose::operator*=(const TwoDPose& other) {
    *this = *this * other;
    return *this;
}