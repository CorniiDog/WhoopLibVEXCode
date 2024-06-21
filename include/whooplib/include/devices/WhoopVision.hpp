#ifndef WHOOP_VISION_HPP
#define WHOOP_VISION_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/calculators/TwoDPose.hpp"
#include "vex.h"
#include <vector>
#include <memory>


struct Pose{
    double x=0, y=0, z=0;
    double pitch=0, yaw=0, roll=0;
};

enum tare_rest{
    do_tare=true,
    dont_tare=false
};

// Declaration of WhoopDrivetrain class
class WhoopVision {
protected:
    // Upon initialization
    std::unique_ptr<Messenger> pose_messenger = nullptr;
    // These functions are ran automatically
    void _transform_pose();
    void _update_pose(std::string pose_data);

    Pose raw_pose;

    // Tares
    double tare_x = 0;
    double tare_y = 0;
    double tare_z = 0;
    double tare_pitch = 0;
    double tare_roll = 0;
    double tare_yaw = 0;

    // Tared computes
    double tared_z = this->raw_pose.z - tare_z;
    double tared_pitch = this->raw_pose.pitch - tare_pitch;
    double tared_roll = this->raw_pose.roll - tare_roll;
    TwoDPose tared_position;

private:
    void setup_messenger(BufferNode* bufferSystem, const std::string& pose_stream);
public:
    vex::mutex thread_lock;  // Mutex for synchronization
    //Pose raw_pose;

    Pose pose;

    // Initialization Constructors
    WhoopVision(BufferNode* bufferSystem, std::string pose_stream); 

    void tare(double x, double y, double z, double pitch, double yaw, double roll);
    void tare(double x, double y, double yaw, tare_rest tare_rest_to_zero);
    void tare(double x, double y, double yaw);
    
    Pose get_pose();
};


#endif // WHOOP_MOTOR_HPP

