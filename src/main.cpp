/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Whooplib Template                                         */
/*                                                                            */
/*    Contributions:                                                          */
/*      2775 Josh:                                                            */
/*        https://github.com/JacksonAreaRobotics/JAR-Template                 */
/*      Intel:                                                                */
/*        https://github.com/IntelRealSense/librealsense                      */
/*      PiLons:                                                               */
/*        http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf         */
/*      Andrew Walker:                                                        */
/*        https://github.com/AndrewWalker/Dubins-Curves/tree/master           */
/*      Alex:                                                                 */
/*        https://www.learncpp.com/                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "whooplib.h"

using namespace vex;

// A global instance of competition
competition Competition;

////////////////////////////////////////////////////////////
/**
 *    Globals
 */
////////////////////////////////////////////////////////////

// Primary controller
WhoopController controller1(joystickmode::joystickmode_split_arcade, controllertype::controller_primary);

// Left drive motors
WhoopMotor l1(PORT12, cartridge::blue, reversed::yes_reverse);
WhoopMotor l2(PORT13, cartridge::blue, reversed::yes_reverse);
WhoopMotor l3(PORT14, cartridge::blue, reversed::yes_reverse);
WhoopMotor l4(PORT15, cartridge::blue, reversed::yes_reverse);
WhoopMotorGroup left_motors({&l1, &l2, &l3, &l4});

// Right drive motors
WhoopMotor r1(PORT1, cartridge::blue, reversed::no_reverse);
WhoopMotor r2(PORT2, cartridge::blue, reversed::no_reverse);
WhoopMotor r3(PORT3, cartridge::blue, reversed::no_reverse);
WhoopMotor r4(PORT4, cartridge::blue, reversed::no_reverse);
WhoopMotorGroup right_motors({&r1, &r2, &r3, &r4});

// Sensors
WhoopInertial inertial_sensor(PORT7);
WhoopRotation forward_tracker(PORT6, reversed::no_reverse);
WhoopRotation sideways_tracker(PORT9, reversed::no_reverse);

// ////////////////////////////////////////////////////////////
// /**
//  *    Wheel Odometry Configuration
//  */
// ////////////////////////////////////////////////////////////

WhoopDriveOdomUnit odom_unit(
    1.51_in,          // The forward tracker distance from the odom unit's center. (positive implies a shift to the right from the odom unit's center)
    2.5189_in,        // Diameter of the forward tracker (e.g., 3.25_in for 3.25-inch wheels).
    -4.468_in,        // The sideways tracker distance from the odom unit's center (positive implies a shift forward from the odom unit center)
    2.5189_in,        // Diameter of the sideways tracker (e.g., 3.25_in for 3.25-inch wheels).
    &inertial_sensor, // Pointer to the WhoopInertial sensor
    &forward_tracker, // Pointer to the forward tracker, as a WhoopRotation sensor
    &sideways_tracker // Pointer to the sideways tracker, as a WhoopRotation sensor
);

WhoopDriveOdomOffset odom_offset(
    &odom_unit, // Pointer to the odometry unit (will manage the odom unit)
    -0.6_in,    // The x offset of the odom unit from the center of the robot (positive implies a shift right from the center of the robot).
    4.95_in     // The y offset of the odom unit from the center of the robot (positive implies a shift forward from the center of the robot).
);

////////////////////////////////////////////////////////////
/**
 *    Wheel Odometry Fusion (No Vision System)
 */
////////////////////////////////////////////////////////////
WhoopOdomFusion odom_fusion(
    &odom_offset
);

////////////////////////////////////////////////////////////
/**
 *    Pure Pursuit Default Parameters
 */
////////////////////////////////////////////////////////////

PursuitParams pursuit_parameters(
    /////////////////////////
    // Path Generation
    /////////////////////////
    // Radius of the turns
    5_in 

    /////////////////////////
    // Pure Pursuit
    /////////////////////////
    // Pure Pursuit look ahead distance
    ,5_in
    // The number of points when generating the path. More points mean higher detail of the path, but at a higher computational cost
    ,100_points 

    /////////////////////////
    // Motor Voltages
    /////////////////////////
    // Pure pursuit forward max motor voltage (0.0, 12.0]
    ,8.0_volts
    // Pure pursuit turning max motor voltage (0.0, 12.0]
    ,12.0_volts

    /////////////////////////
    // Settling
    /////////////////////////
    // Settle Distance. Exits when within this distance of target
    ,1.25_in
    // Settle Rotation. Exits when within this rotation of target
    ,1.1_deg
    // Minimum time to be considered settled, in seconds
    ,0.0_sec
    // Time after which to give up and move on, in seconds (set to 0 to disable)
    ,0_sec
    
    /////////////////////////
    // Turning PID
    /////////////////////////
    // Turning (kP) Proportional Tuning
    ,14_kp
    // Turning (kI) Integral Tuning
    ,0.2_ki
    // Turning (kD) Derivative Tuning
    ,95.0_kd
    // Turning (kR) Integral anti-windup Tuning. Higher value implies greater anti-windup near error=0. 
    // NOTE: Affected by turning_i_activation
    ,1.0_kr
    // The rotation distance (error) to activate turning_ki
    ,20.0_deg
    // The maximum turning voltage change per second, as a slew rate
    ,250.0_volts

    /////////////////////////
    // Forward PID
    /////////////////////////
    // Forward (kP) Proportional Tuning
    ,50.0_kp
    // Forward (kI) Integral Tuning
    ,0.1_ki
    // Forward (kD) Derivative Tuning
    ,250.0_kd
    // Forward (kR) Integral anti-windup Tuning. Higher value implies greater anti-windup near error=0. 
    // NOTE: Affected by forward_i_activation
    ,0.0_kr
    // The forward distance (error) to activate forward_ki
    ,2.0_in
    // The maximum forward voltage change per second, as a slew rate
    ,150.0_volts
);

////////////////////////////////////////////////////////////
/**
 *    Robot Drivetrain and Manager
 */
////////////////////////////////////////////////////////////
WhoopDrivetrain robot_drivetrain(
    &pursuit_parameters,  // The default pure pursuit parameters for operating the robot in autonomous
    &odom_fusion,         // Odometry fusion module
    PoseUnits::in_deg_cw, // Set default pose units if not defined. "m_deg_cw" means "meters, degrees, clockwise-positive yaw", "in_deg_ccw" means "inches, degrees, counter-clockwise-positive yaw", and so forth.
    &controller1,         // Pointer to the controller
    &left_motors,         // Pointer to the left motor group (optionally can be a list of motors as well)
    &right_motors         // Pointer to the right motor group (optionally can be a list of motors as well)
);

/**
 * My first autonomous routine
 */
void auton_1(){
    robot_drivetrain.set_pose_units(PoseUnits::in_deg_cw);
    robot_drivetrain.set_pose(0, 0, 0);

    // robot_drivetrain.turn_to_position(15, 15);
    robot_drivetrain.drive_forward(15);

    robot_drivetrain.turn_to(90);

    robot_drivetrain.drive_forward(-15);

    robot_drivetrain.drive_forward(15);

    robot_drivetrain.turn_to(0);

    robot_drivetrain.drive_forward(-15);

    // robot_drivetrain.drive_to_point(15, 15);
    // robot_drivetrain.reverse_to_point(0,0);
    robot_drivetrain.drive_through_path({{15, 15, 0}, {0, 0, 90}}, 7);
    robot_drivetrain.reverse_through_path({{15, 15, 180}, {0, 0, 180}}, 7);
}

/**
 * My second autonomous routine
 */
void auton_2(){

}

/**
 * My third autonomous routine
 */
void auton_3(){

}

WhoopAutonSelector auton_selector(&controller1, {
    AutonRoutine("First Auton", auton_1),
    AutonRoutine("Second Auton", auton_2),
    AutonRoutine("Third Auton", auton_3)
}, "auton.txt");

ComputeManager manager({&robot_drivetrain, &controller1, &auton_selector});

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton()
{
    robot_drivetrain.set_state(drivetrainState::mode_disabled);
    auton_selector.run_selector();

    // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();
    controller1.notify("Initializing");
    manager.start();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous()
{
    robot_drivetrain.set_state(drivetrainState::mode_autonomous);
    auton_selector.run_autonomous();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void usercontrol()
{
  robot_drivetrain.set_state(drivetrainState::mode_usercontrol);

  // User control code here, inside the loop
  while (true)
  {
    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true)
    wait(100, msec);
}
