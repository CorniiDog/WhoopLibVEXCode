/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
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

WhoopInertial inertial_sensor(PORT7);

// Right drive motors
WhoopMotor r1(PORT1, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor r2(PORT2, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor r3(PORT3, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor r4(PORT4, gearSetting::ratio6_1, reversed::yes_reverse);

// Left drive motors
WhoopMotor l1(PORT12, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor l2(PORT13, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor l3(PORT14, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor l4(PORT15, gearSetting::ratio6_1, reversed::no_reverse);

// Robot Offset 
// First variable is x, which +x is the direction of right from the center of the robot
// Second variable is y, which +y is the direction of forwardness from the center of the robot (in meters)
double x_offset = 0;
double y_offset = 15.0/100.0;
RobotVisionOffset vision_offset(x_offset, y_offset);

// Jetson Nano pose retreival stream identifier (configured on Nano-side) 
std::string pose_stream = "P";
WhoopVision vision_system(&vision_offset, &buffer_system, pose_stream);

//Gear ratio on the drivetrain (If it's 32t driving 64t, it would be a 1.0/2.0 gear ratio, or 0.5)
double gear_ratio = 1.0/2.0;

// Wheel diameter (converted from inches to meters)
double wheel_diameter = to_meters(2.9845); 

WhoopDrivetrain robot_drivetrain(gear_ratio, &controller1, {&l1, &l2, &l3, &l4}, {&r1, &r2, &r3, &r4});

ComputeManager manager({&buffer_system, &robot_drivetrain});

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

  inertial_sensor.calibrate();

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

  Brain.Screen.print("Move Robot in 5s");
  wait(10, sec);
  vision_system.tare(1, 1, M_PI/4);


  // User control code here, inside the loop
  while (1) {


    Brain.Screen.clearLine(2);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Pose: %.3f %.3f %.3f %.3f %.3f %.3f", vision_system.pose.x, vision_system.pose.y, vision_system.pose.z, vision_system.pose.pitch, vision_system.pose.yaw, vision_system.pose.roll);
    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Inertial: %.3f", inertial_sensor.get_yaw_radians());
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
