/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PurePursuit.hpp                                           */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 3 2024                                           */
/*    Description:  Pure Pursuit Steering Calculator                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "whooplib/include/calculators/Dubins.h"
#include "whooplib/include/calculators/TwoDPose.hpp"


struct PursuitEstimate{
    bool is_valid = false;
    double steering_angle = 0;
    double distance = 0;
    PursuitEstimate(bool is_valid=false, double steering_angle=0, double distance=0): is_valid(is_valid), steering_angle(steering_angle), distance(distance){}
};

class PurePursuitPath {
    TwoDPose start, end;
    double turning_radius;
    double lookahead_distance;
    double q0[3];
    double q1[3];
    DubinsPath path;
    bool path_valid;
    double t_max = 0;
    double num_segments;
    double step_size;

private:
    void computeDubinsPath();

public:
    /**
     * Creates a path for pure pursuit, using Dubin-Curves.
     * You can find more information about Dubin-Curves here: https://github.com/AndrewWalker/Dubins-Curves?tab=readme-ov-file 
     * @param start The TwoDPose object representing the start of the path (like robot position)
     * @param end The TwoDPose object representing the end of the path (pose to drive to)
     * @param turning_radius Turning radius of the dubin curve, in meters. You can see visual representations of the turning radius here: https://imgur.com/BahIst0 
     * @param lookahead_distance The look-ahead distance for pure pursuit along a path, in meters. Visual representation of lookahead distance here: https://imgur.com/WT5G0Z1
     * @param num_segments The number of points when generating the path. More points mean higher detail of the path, but at a higher computational cost
     */
    PurePursuitPath(const TwoDPose start, const TwoDPose end, double turning_radius, double lookahead_distance, double num_segments=200);

    /**
     * Calculates the pure pursuit estimate relative to the path.
     * @param current_position The TwoDPose of the current position
     * @param find_closest_if_off_course Set to true to find the closest point of the path
     * @returns PursuitEstimate, which contains "is_valid" bool which returns true if succeeded successfully, or false if the robot is off the path.
     * Also includes "steering_angle" which is the angle to steer to (if + means steer left, if - means steer right).
     * "distance" is how far away from the lookahead point.
     */
    PursuitEstimate calculate_pursuit_estimate(TwoDPose current_position, bool find_closest_if_off_course=true);
};



#endif // PURE_PURSUIT_HPP