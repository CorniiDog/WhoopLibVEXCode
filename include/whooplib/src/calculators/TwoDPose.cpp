
#include "whooplib/include/calculators/TwoDPose.hpp"
#include <cmath>

TwoDPose::TwoDPose(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}

TwoDPose TwoDPose::operator*(const TwoDPose& other) const {
    // Calculate the new position
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    double new_x = x + other.x * cos_yaw - other.y * sin_yaw;
    double new_y = y + other.x * sin_yaw + other.y * cos_yaw;

    // Calculate the new yaw
    double new_yaw = yaw + other.yaw;

    // Normalize the yaw to the range [-pi, pi]
    new_yaw = fmod(new_yaw + M_PI, 2 * M_PI) - M_PI;
    if (new_yaw < -M_PI) {
        new_yaw += 2 * M_PI;
    }

    return TwoDPose(new_x, new_y, new_yaw);
}

TwoDPose TwoDPose::toObjectSpace(const TwoDPose& other) const {
    // Calculate the relative position
    double dx = other.x - x;
    double dy = other.y - y;

    // Create the rotation matrix for -yaw
    double cos_yaw = cos(-yaw);
    double sin_yaw = sin(-yaw);
    
    // Apply the rotation matrix
    double relative_x = dx * cos_yaw - dy * sin_yaw;
    double relative_y = dx * sin_yaw + dy * cos_yaw;

    // Calculate the relative yaw
    double relative_yaw = other.yaw - yaw;

    // Normalize the yaw to the range [-pi, pi]
    relative_yaw = fmod(relative_yaw + M_PI, 2 * M_PI) - M_PI;
    if (relative_yaw < -M_PI) {
        relative_yaw += 2 * M_PI;
    }

    return TwoDPose(relative_x, relative_y, relative_yaw);
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

    // Normalize the yaw to the range [-pi, pi]
    relative_yaw = fmod(relative_yaw + M_PI, 2 * M_PI) - M_PI;
    if (relative_yaw < -M_PI) {
        relative_yaw += 2 * M_PI;
    }

    return TwoDPose(relative_x, relative_y, relative_yaw);
}
