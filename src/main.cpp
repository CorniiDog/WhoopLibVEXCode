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

using namespace vex;

// A global instance of competition
competition Competition;

BufferNode buffer_system(256, false, "/dev/serial1"); // set to false for competition

WhoopController controller1(joystickMode::joystickmode_split_arcade);

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

// Jetson Nano Pose Retreival Channel
std::string pose_channel = "P";
WhoopDrivetrain robot_drivetrain(&buffer_system, pose_channel, &controller1, {&l1, &l2, &l3, &l4}, {&r1, &r2, &r3, &r4});

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

  while(true){
    Brain.Screen.clearLine(4);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Pose (e): %d", robot_drivetrain.pose.yaw);
    wait(20, timeUnits::msec);
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

  // User control code here, inside the loop
  while (1) {

    Brain.Screen.clearLine(4);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Pose (e): %d", robot_drivetrain.pose.yaw);

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
