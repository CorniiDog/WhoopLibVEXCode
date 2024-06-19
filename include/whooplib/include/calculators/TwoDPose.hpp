#ifndef TWODPOSE_HPP
#define TWODPOSE_HPP

class TwoDPose {
public:
    double x, y, yaw;

    TwoDPose(double x, double y, double yaw);

    // Overloaded * operator to combine two poses
    TwoDPose operator*(const TwoDPose& other) const;

    // Method to compute the relative pose
    TwoDPose toObjectSpace(const TwoDPose& other) const;
};

#endif // TWODPOSE_HPP
