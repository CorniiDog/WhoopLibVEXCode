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
    end_pushed_back = end * TwoDPose(0,-lookahead_distance,0);

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

    q1[0] = end_pushed_back.x;
    q1[1] = end_pushed_back.y;
    q1[2] = end_pushed_back.yaw;

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
        
        // Create extrapolated forward steps
        double dx = end.x - end_pushed_back.x;
        double dy = end.y - end_pushed_back.y;
        double distance = sqrt(dx * dx + dy * dy);
        t_max += distance;
        int n = static_cast<int>(distance / step_size); // Calculate the number of full steps
        double fraction_step = step_size / distance;

        for (int i = 1; i < n; ++i) {
            double fraction = i * fraction_step;
            barebonesPose intermediate;
            intermediate.x = end_pushed_back.x + fraction * dx;
            intermediate.y = end_pushed_back.y + fraction * dy;
            pursuit_points.push_back(intermediate);
        }

    }
}

int PurePursuitPath::create_points(double q[3], double x)
{
    barebonesPose p(q[0], q[1], q[2]);
    pursuit_points.push_back(p);
    return 0;
}

PursuitEstimate PurePursuitPath::calculate_pursuit_estimate(TwoDPose current_position, bool find_closest_if_off_course, double deviation_min)
{
    if (!path_valid)
    {
        return PursuitEstimate();
    }

    barebonesPose look_ahead_position;
    barebonesPose closest_position;
    bool lookahead_found = false;
    bool closest_found = false;

    double point_ahead_distance = lookahead_distance; // Distance from the current position to lookahead position
    double closest_distance = std::numeric_limits<double>::max();

    double length_lookahead = 0; // The distance from the lookahead position to the point around the curve
    double length_closest = 0;

    int points_size = pursuit_points.size();

    double rough_distance;
    double distance;

    // Reverse iteration
    for (std::size_t i = points_size; i-- > 0; )
    {
        // Rough distance first to avoid un-needed computational cost
        rough_distance = (std::abs(pursuit_points[i].x - current_position.x) + std::abs(pursuit_points[i].y - current_position.y)) / 2;
        if (rough_distance > closest_distance)
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
                length_lookahead = (points_size - 1 - i) * step_size;
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
                length_closest = (points_size - 1 - i) * step_size;
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


    lookahead_pos = look_ahead_position;

    bool is_past_point = false;

    if(length_lookahead == 0) {
        is_past_point = true;
        // If robot is within a respective distance, then just turn
        /*double dist_from_end = std::sqrt(std::pow(q1[0] - current_position.x, 2)+ std::pow(q1[1] - current_position.y, 2));
        if(dist_from_end < deviation_min){
            steering_angle = normalize_angle(q1[2] - current_position.yaw);
        }*/

        // If the angle of the robot at the end point, facing the same direction roughly as the end point, and over-passes (designated by steering angle)
        // Go in reverse
        if(std::abs(normalize_angle(q1[2] - current_position.yaw)) < M_PI_2 && std::abs(steering_angle) > M_PI_2){
            point_ahead_distance *= -1;
            steering_angle = normalize_angle(steering_angle + M_PI);
        }
    }
    

    return PursuitEstimate(true, steering_angle, point_ahead_distance + length_lookahead, is_past_point);
}
