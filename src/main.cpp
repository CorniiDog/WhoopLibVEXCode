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

// Jetson Nano pose retreival stream identifier (configured on Nano-side) 
std::string pose_stream = "P";

//Gear ratio on the drivetrain (If it's 32t driving 64t, it would be a 1.0/2.0 gear ratio, or 0.5)
double gear_ratio = 1.0/2.0;

// Wheel diameter (converted from inches to meters)
double wheel_diameter = to_meters(2.9845); 

WhoopDrivetrain robot_drivetrain(gear_ratio, &buffer_system, pose_stream, &controller1, {&l1, &l2, &l3, &l4}, {&r1, &r2, &r3, &r4});

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

  double tare_x = 0;
  double tare_y = 0;
  double tare_z = 0;
  double tare_pitch = 0;
  double tare_roll = 0;
  double tare_yaw = 0;

  TwoDPose tared_position(robot_drivetrain.pose.x, robot_drivetrain.pose.y, robot_drivetrain.pose.yaw - tare_yaw);

  double tared_z = robot_drivetrain.pose.z - tare_z;
  double tared_pitch = robot_drivetrain.pose.pitch - tare_pitch;
  double tared_roll = robot_drivetrain.pose.roll - tare_roll;

  // User control code here, inside the loop
  while (1) {

    TwoDPose transposed = tared_position.toObjectSpace(robot_drivetrain.pose.x, robot_drivetrain.pose.y, robot_drivetrain.pose.yaw);

    double current_x = transposed.x + tare_x;
    double current_y = transposed.y + tare_y;
    double current_z = tared_z;
    double current_pitch = tared_pitch;
    double current_yaw = transposed.yaw;
    double current_roll = tared_roll;

    Brain.Screen.clearLine(2);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Pose: %.3f %.3f %.3f %.3f %.3f %.3f", current_x, current_y, current_z, current_pitch, current_yaw, current_roll);
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
