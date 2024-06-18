#ifndef WHOOP_DRIVETRAIN_HPP
#define WHOOP_DRIVETRAIN_HPP

#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "vex.h"
#include <vector>
#include <memory>

enum drivetrainState{
    mode_disabled=1,
    mode_autonomous=2,
    mode_usercontrol=3
};

struct Pose{
    double x=0, y=0, z=0;
    double pitch=0, yaw=0, roll=0;
};

// Declaration of WhoopDrivetrain class
class WhoopDrivetrain : public ComputeNode  {
protected:
    // Upon initialization
    WhoopController* whoop_controller;
    std::unique_ptr<WhoopMotorGroup> left_motor_group;
    std::unique_ptr<WhoopMotorGroup> right_motor_group;

    drivetrainState drive_state = drivetrainState::mode_disabled;

    Messenger* pose_messenger = nullptr;

    // These functions are ran automatically
    void _update_pose(std::string pose_data);
public:
    Pose pose;
    // Initialization Constructors
    WhoopDrivetrain(Messenger* messenger, WhoopController* controller,  WhoopMotorGroup* leftMotorGroup, WhoopMotorGroup* rightMotorGroup); 
    WhoopDrivetrain(Messenger* messenger, WhoopController* controller, std::vector<WhoopMotor*> left_motors, std::vector<WhoopMotor*> right_motors); 

    void set_state(drivetrainState state);

    Pose get_pose();
protected:
    void __step() override;  // Protected helper function for processing steps
};


#endif // WHOOP_MOTOR_HPP

