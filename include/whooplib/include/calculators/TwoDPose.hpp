/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       TwoDPose.hpp                                              */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Two-Dimensional CFrames                                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef TWODPOSE_HPP
#define TWODPOSE_HPP

#include <cmath>
#include <sstream>
#include <iomanip>
#include <string>

/**
 * Represents a 2D pose with position and orientation in a Cartesian coordinate system.
 * Yaw is in radians, counter-clockwise.
 * +x is right-face direction, +y is front-face direction
 * The rest of the standardization is up to the end-user to decide.
 */
class TwoDPose {
public:
    double x, y, yaw; // X and Y positions, and yaw (orientation) in radians.

    /**
     * Constructs a TwoDPose object representing a position and orientation in 2D space.
     * @param x The x-coordinate of the pose, where positive values indicate rightward movement.
     * @param y The y-coordinate of the pose, where positive values indicate forward movement.
     * @param yaw The orientation of the pose in radians, where positive values indicate counter-clockwise rotation.
     */
    TwoDPose(double x, double y, double yaw);

    // Overloaded * operator to combine two poses
    // Think of it as Roblox's CF = CFrame1 * CFrame2
    /**
     * Combines this pose with another pose using matrix multiplication semantics.
     * Equivalent to applying the transformation of the other pose to this pose.
     * @param other Another TwoDPose to combine with this pose.
     * @return A new TwoDPose representing the combined transformation.
     */
    TwoDPose operator*(const TwoDPose& other) const;

    // This allows for *= as well
    /**
     * Applies and assigns the transformation of another pose to this pose.
     * @param other Another TwoDPose whose transformation is applied to this pose.
     * @return A reference to this pose after the transformation.
     */
    TwoDPose& operator*=(const TwoDPose& other);

    
    // Returns a TwoDPose that represents the delta change applied to the *=, relative to the first TwoDPose
    /**
     * Calculates the transformation required to go from this pose to another pose.
     * This can be thought of as the relative movement and rotation from this pose to the specified pose.
     * @param other The pose to which the transformation is calculated.
     * @return A TwoDPose representing the required transformation.
     */
    TwoDPose multiplicative_delta(const TwoDPose& other) const;

    // Method to compute the relative pose
    // Think of it as Roblox's CFrame1:ToObjectSpace(CFrame2)
    // in which it returns the pose of CFrame2 relative to CFrame1
    // But in this case, TwoDPose other relative to TwoDPose self
    /**
     * Transforms another pose into the coordinate space of this pose.
     * This is analogous to calculating the pose of an object relative to a reference frame defined by this pose.
     * @param other The pose to transform into this pose's coordinate space.
     * @return A new TwoDPose representing the pose relative to this pose.
     */
    TwoDPose toObjectSpace(const TwoDPose& other) const;

    // Method to retreive world space from object space
    // Example of getting object space:
    // TwoDPose b_in_a_object_space = a.toObjectSpace(b);
    // You can then recover b back with:
    // TwoDPose b_recovered = a.toWorldSpace(b_in_a_object_space);
    /**
     * Transforms a pose from the local coordinate space of this pose back into the global coordinate space.
     * This is the inverse operation of toObjectSpace, effectively recalculating the global coordinates of a pose
     * that has been transformed into this pose's local space. It re-applies the global positioning and orientation
     * of this pose to the local pose.
     * @param other The pose in this object's local coordinate space to transform back to global space.
     * @return A new TwoDPose representing the pose in global coordinates.
     */
    TwoDPose toWorldSpace(const TwoDPose& other) const;

    // Method to compute the inverse of the pose
    /**
     * Computes the inverse of this pose, effectively creating a pose that,
     * when multiplied by this pose, yields the identity pose (zero translation and zero rotation).
     * This is used to revert transformations or to calculate relative transformations between two poses.
     * @return A new TwoDPose representing the inverse of this pose.
     */
    TwoDPose inverse() const;

    // Method to multiply by the inverse of another pose
    /**
     * Multiplies this pose by the inverse of another pose. This is useful for calculating
     * the relative transformation from this pose to another pose, effectively undoing the other pose's transformation.
     * @param other The pose whose inverse is multiplied with this pose.
     * @return A new TwoDPose representing the result of the multiplication.
     */
    TwoDPose inverseMultiply(const TwoDPose& other) const;

    // Unary negation operator
    TwoDPose operator-() const;

    // Overloaded / operator to divide one pose by another
    /**
     * Divides this pose by another pose using the inverse of the other pose.
     * This operation is equivalent to multiplying this pose by the inverse of the other pose,
     * used to calculate the pose that, when multiplied by `other`, would result in this pose.
     * @param other The pose to divide by.
     * @return A new TwoDPose representing the division of this pose by the other pose.
     */
    TwoDPose operator/(const TwoDPose& other) const;

    // This allows for /= as well
    /**
     * Applies and assigns the division of this pose by another pose.
     * This is essentially the multiplication of this pose by the inverse of the other pose,
     * and is used to adjust this pose by the inverse of the transformations applied by `other`.
     * @param other The pose by which this pose is divided.
     * @return A reference to this pose after the division.
     */
    TwoDPose& operator/=(const TwoDPose& other);
    
    // Method to compute relative pose of just given x y and yaw.
    // This is useful for tare.
    /**
     * Transforms given position and orientation coordinates into the coordinate space of this pose.
     * Useful for recalibrating or 'taring' sensors relative to a current pose.
     * @param x X-coordinate to transform.
     * @param y Y-coordinate to transform.
     * @param yaw Orientation in radians to transform.
     * @return A new TwoDPose representing the transformed coordinates.
     */
    TwoDPose toObjectSpace(double x, double y, double yaw) const;

    /**
     * Returns a string representation of the pose with the coordinates rounded to a specified number of decimal places.
     * If decimal_places is -1, no rounding is applied.
     * @param decimal_places The number of decimal places to round to, or -1 for no rounding.
     * @return A string in the format "x y yaw".
     */
    std::string to_string(int decimal_places=4);

    /**
     * Returns a string representation of the pose with the coordinates rounded to a specified number of decimal places.
     * This variant follows pose standard for the T265
     * If decimal_places is -1, no rounding is applied.
     * @param decimal_places The number of decimal places to round to, or -1 for no rounding.
     * @return A string in the format "x y yaw".
     */
    std::string to_realsense_string(int decimal_places=4);
};

#endif // TWODPOSE_HPP
