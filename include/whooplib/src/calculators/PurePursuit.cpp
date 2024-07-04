/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PurePursuit.cpp                                           */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 3 2024                                           */
/*    Description:  Pure Pursuit Steering Calculator                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/PurePursuit.hpp"
#include "whooplib/include/toolbox.hpp"

PurePursuitPath::PurePursuitPath(const TwoDPose start, const TwoDPose end, double turning_radius, double lookahead_distance,double num_segments) 
    : start(start), end(end), turning_radius(turning_radius), lookahead_distance(lookahead_distance), num_segments(num_segments){
    computeDubinsPath();
}

void PurePursuitPath::computeDubinsPath() {
    q0[0] = start.x;
    q0[1] = start.y;
    q0[2] = start.yaw;

    q1[0] = end.x;
    q1[1] = end.y;
    q1[2] = end.yaw;

    path_valid = (dubins_shortest_path(&path, q0, q1, turning_radius) == EDUBOK);

    if(path_valid){
        t_max = dubins_path_length(&path);
        step_size = t_max/num_segments;
    }
}

PursuitEstimate PurePursuitPath::calculate_pursuit_estimate(TwoDPose current_position, bool find_closest_if_off_course) {
    if(!path_valid){
        return PursuitEstimate();
    }
    double point_ahead_distance = lookahead_distance;
    double closest_t = 0.0;
    double q[3];  // Sample point on the path
    bool found = false;

    double rough_distance;
    double distance;

    // Reverse iteration
    for (double t = t_max; t >= 0; t -= step_size) {
        if(dubins_path_sample(&path, t, q) != EDUBOK){
            continue;  // Skip if there's an error in sampling
        }

        // Rough distance first to avoid un-needed computational cost
        rough_distance = (std::abs(q[0] - current_position.x) + std::abs(q[1] - current_position.y))/2;
        if (rough_distance > point_ahead_distance){
            continue;
        }

        distance = sqrt(pow(q[0] - current_position.x, 2) + pow(q[1] - current_position.y, 2));
        if (distance <= point_ahead_distance) {
            point_ahead_distance = distance; // Replace, as point_ahead_distance
            closest_t = t;
            found = true;
            break;  // Break on finding the look ahead point
        } 
    }

    if (!found) {
        if(!find_closest_if_off_course){
            return PursuitEstimate();  // Return invalid pursuit estimate if no point is found
        }
        else{ // Find closest point and turn to
           point_ahead_distance = std::numeric_limits<double>::max();
           for (double t = t_max; t >= 0; t -= step_size) {
                if(dubins_path_sample(&path, t, q) != EDUBOK){
                    continue;  // Skip if there's an error in sampling
                }

                // Rough distance first to avoid un-needed computational cost
                rough_distance = (std::abs(q[0] - current_position.x) + std::abs(q[1] - current_position.y))/2;
                if (rough_distance > point_ahead_distance){
                    continue;
                }

                distance = sqrt(pow(q[0] - current_position.x, 2) + pow(q[1] - current_position.y, 2));
                if (distance <= point_ahead_distance) {
                    point_ahead_distance = distance; // Replace, as point_ahead_distance
                    closest_t = t;
                    found = true;
                } 
            }

            if(!found){ // If still not found, return empty
                return PursuitEstimate();  // Return invalid pursuit estimate if no point is found
            }
        }
    }

    double dx = q[0] - current_position.x;
    double dy = q[1] - current_position.y;
    double path_angle = atan2(dy, dx);
    double steering_angle = path_angle - current_position.yaw;
    steering_angle = atan2(sin(steering_angle), cos(steering_angle));  // Normalize angle

    return PursuitEstimate(true, steering_angle, point_ahead_distance);
}
