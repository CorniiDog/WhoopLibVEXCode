#ifndef TWODPOSE_HPP
#define TWODPOSE_HPP

class TwoDPose {
public:
    double x, y, yaw;

    TwoDPose(double x, double y, double yaw);

    // Overloaded * operator to combine two poses
    // Think of it as Roblox's CF = CFrame1 * CFrame2
    TwoDPose operator*(const TwoDPose& other) const;

    // Method to compute the relative pose
    // Think of it as Roblox's CFrame1:ToObjectSpace(CFrame2)
    // in which it returns the pose of CFrame2 relative to CFrame1
    // But in this case, TwoDPose other relative to TwoDPose self
    TwoDPose toObjectSpace(const TwoDPose& other) const;
    
    // Method to compute relative pose of just given x y and yaw.
    // This is useful for tare.
    TwoDPose toObjectSpace(double x, double y, double yaw) const;
};

#endif // TWODPOSE_HPP
