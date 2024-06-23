/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Whooplib Template                                         */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "whooplib.h"
#include <iostream>
#include <sstream>

using namespace vex;

// A global instance of competition
competition Competition;

// Serial communication module
BufferNode buffer_system(256, debugMode::debug_disabled, "/dev/serial1"); // set to debug_disabled for competition

WhoopController controller1(joystickMode::joystickmode_split_arcade);

// Right drive motors
WhoopMotor r1(PORT1, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor r2(PORT2, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor r3(PORT3, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor r4(PORT4, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotorGroup right_motors({&r1, &r2, &r3, &r4});

// Left drive motors
WhoopMotor l1(PORT12, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor l2(PORT13, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor l3(PORT14, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor l4(PORT15, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotorGroup left_motors({&l1, &l2, &l3, &l4});

// Vision Offset from Center of Robot
// First variable is x, which +x is the direction of right from the center of the robot in meters
// Second variable is y, which +y is the direction of forwardness from the center of the robot in meters
double x_offset = 0;
double y_offset = 15.0/100.0;
RobotVisionOffset vision_offset(x_offset, y_offset);

// Jetson Nano pose retreival stream identifier (configured on Nano-side) 
std::string pose_stream = "P";
WhoopVision vision_system(&vision_offset, &buffer_system, pose_stream);

//Gear ratio on the drivetrain (If it's a [motor powering the 32t] driving the [64t w/ shared axle to the wheels], it would be a 1.0/2.0 gear ratio, or 0.5)
double gear_ratio = 1.0/2.0;

// Wheel diameter in meters
double wheel_diameter_meters = to_meters(3); 

WhoopInertial inertial_sensor(PORT7);
WhoopRotation forward_tracker(PORT6, reversed::yes_reverse);
WhoopRotation sideways_tracker(PORT9, reversed::yes_reverse);

double forward_wheel_diameter_meters = to_meters(2.5189); // (e.g., 0.08255 for 3.25-inch wheels).
double sideways_wheel_diameter_meters = to_meters(2.5189);

/**
 * SUGGESTION: It is easier and likely more precise to measure the distances by placing the robot over a glass table to allow the odometry wheels to recess to default 
 * position and measure from below.
 */

// The odom unit center is the virtual intercept of the perpendicular faces of the odometry trackers.
// The measurement is from the center of the odom unit to the designated tracker distances.
// Visual Representation of Tracker Distances from Odom Unit: https://imgur.com/rWCCCfz
double forward_tracker_distance_meters = to_meters(1.29); // Distance from the odom unit center to the forward tracker, in meters (positive implies a shift to the right from the odom unit center).
double sideways_tracker_distance_meters = to_meters(-4.50); // Distance from the odom unit center to the sideways tracker, in meters (positive implies a shift forward from the odom unit center).
WhoopDriveOdomUnit odom_unit(forward_tracker_distance_meters, forward_wheel_diameter_meters, sideways_tracker_distance_meters, sideways_wheel_diameter_meters, &inertial_sensor, &forward_tracker, &sideways_tracker);

// If your Odom Unit's Center is NOT the center of the robot, apply the offset here.
// The measurement is from the center of the robot to the odom unit center.
// If your Odom Unit's Center IS the center of the robot, set to 0,0.
// Visual representation of Odom Unit from Center of Robot: https://imgur.com/x8ObCIG
// TODO: Create Odom Unit Offset Object with x and y


WhoopDrivetrain robot_drivetrain(&controller1, &left_motors, &right_motors);

ComputeManager manager({&buffer_system, &robot_drivetrain, &odom_unit});


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  manager.start();
  robot_drivetrain.set_state(drivetrainState::mode_disabled);

  wait(0.5, sec);

  odom_unit.calibrate();

  while (robot_drivetrain.drive_state == drivetrainState::mode_disabled){
    // Do stuff like calibrate IMU
    wait(20, msec); // Sleep the task for a short amount of time to
  }

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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

void autonomous(void) {
  robot_drivetrain.set_state(drivetrainState::mode_autonomous);
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
void usercontrol(void) {
  robot_drivetrain.set_state(drivetrainState::mode_usercontrol);

  wait(0.5, sec);
  vision_system.tare(1, 1, M_PI/4);


  // User control code here, inside the loop
  while (1) {


    Brain.Screen.clearLine(2);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Pose: %.3f %.3f %.3f %.3f %.3f %.3f", vision_system.pose.x, vision_system.pose.y, vision_system.pose.z, vision_system.pose.pitch, vision_system.pose.yaw, vision_system.pose.roll);
    
    
    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Controller: %.3f", controller1.get_left_joystick_x());

    Brain.Screen.clearLine(4);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Wheel Odom: %.3f %.3f %.3f", odom_unit.pose.x, odom_unit.pose.y, odom_unit.pose.yaw);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) wait(100, msec);
}
