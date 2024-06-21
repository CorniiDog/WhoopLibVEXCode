#ifndef WHOOP_VISION_HPP
#define WHOOP_VISION_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "vex.h"
#include <vector>
#include <memory>


struct Pose{
    double x=0, y=0, z=0;
    double pitch=0, yaw=0, roll=0;
};

// Declaration of WhoopDrivetrain class
class WhoopVision {
protected:
    // Upon initialization

    std::unique_ptr<Messenger> pose_messenger = nullptr;

    // These functions are ran automatically
    void _update_pose(std::string pose_data);

private:
    void setup_messenger(BufferNode* bufferSystem, const std::string& pose_stream);
public:
    vex::mutex thread_lock;  // Mutex for synchronization
    Pose pose;

    // Initialization Constructors
    WhoopVision(BufferNode* bufferSystem, std::string pose_stream); 
    
    Pose get_pose();
};


#endif // WHOOP_MOTOR_HPP

