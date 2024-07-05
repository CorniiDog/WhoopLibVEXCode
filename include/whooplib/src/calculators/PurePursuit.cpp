/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PurePursuit.cpp                                           */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 3 2024                                           */
/*    Description:  Pure Pursuit Steering Calculator                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/PurePursuit.hpp"
#include "whooplib/include/calculators/Dubins.hpp"
#include "whooplib/include/toolbox.hpp"
#include "vex.h"

PurePursuitPath::PurePursuitPath(const TwoDPose start, const TwoDPose end, double turning_radius, double lookahead_distance, double num_segments)
    : start(start), end(end), turning_radius(turning_radius), lookahead_distance(lookahead_distance), num_segments(num_segments)
{
    computeDubinsPath();
}

static int create_points_bridge(double q[3], double t, void *user_data)
{
    return static_cast<PurePursuitPath *>(user_data)->create_points(q, t);
}

void PurePursuitPath::computeDubinsPath()
{
    q0[0] = start.x;
    q0[1] = start.y;
    q0[2] = start.yaw;

    q1[0] = end.x;
    q1[1] = end.y;
    q1[2] = end.yaw;

    path_valid = (dubins_shortest_path(&path, q0, q1, turning_radius) == EDUBOK);

    if (path_valid)
    {
        t_max = dubins_path_length(&path);
        step_size = t_max / num_segments;

        pursuit_points = {};
        if (dubins_path_sample_many(&path, step_size, create_points_bridge, this) != EDUBOK)
        {
            path_valid = false;
        }
    }
}

int PurePursuitPath::create_points(double q[3], double x)
{
    barebonesPose p(q[0], q[1], q[2]);
    pursuit_points.push_back(p);
    return 0;
}

PursuitEstimate PurePursuitPath::calculate_pursuit_estimate(TwoDPose current_position, bool find_closest_if_off_course)
{
    if (!path_valid)
    {
        return PursuitEstimate();
    }
    double point_ahead_distance = lookahead_distance;
    double closest_distance = std::numeric_limits<double>::max();
    barebonesPose look_ahead_position;
    barebonesPose closest_position;
    bool lookahead_found = false;
    bool closest_found = false;

    double length_lookahead = 0;
    double length_closest = 0;

    double rough_distance;
    double distance;

    // Reverse iteration
    for (double i = pursuit_points.size(); i >= 0; --i)
    {
        // Rough distance first to avoid un-needed computational cost
        rough_distance = (std::abs(pursuit_points[i].x - current_position.x) + std::abs(pursuit_points[i].y - current_position.y)) / 2;
        if (rough_distance > point_ahead_distance)
        {
            continue;
        }

        distance = sqrt(pow(pursuit_points[i].x - current_position.x, 2) + pow(pursuit_points[i].y - current_position.y, 2));
        if (distance <= point_ahead_distance)
        {
            if (!lookahead_found)
            {
                point_ahead_distance = distance;
                look_ahead_position = pursuit_points[i];
                length_lookahead = (pursuit_points.size() - i) * step_size;
                lookahead_found = true;
            }
            if (!find_closest_if_off_course)
            {
                break;
            }
        }

        if (find_closest_if_off_course)
        {
            if (distance <= closest_distance)
            {
                closest_distance = distance;
                closest_position = pursuit_points[i];
                length_closest = (pursuit_points.size() - i) * step_size;
                closest_found = true;
            }
        }
    }

    if (!lookahead_found)
    {
        if (!closest_found)
        {
            return PursuitEstimate(); // Return invalid pursuit estimate if no point is found
        }
        look_ahead_position = closest_position;
        point_ahead_distance = closest_distance;
        length_lookahead = length_closest;
    }

    double dx = look_ahead_position.x - current_position.x;
    double dy = look_ahead_position.y - current_position.y;
    double path_angle = atan2(dy, dx);
    double steering_angle = normalize_angle(path_angle - current_position.yaw);

    return PursuitEstimate(true, steering_angle, point_ahead_distance + length_closest);
}
