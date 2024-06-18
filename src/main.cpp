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

Messenger pose_messenger(&buffer_system, "P", deleteAfterRead::no_delete);

std::vector<ComputeNode*> nodes = {&buffer_system};
ComputeManager manager(nodes);

WhoopController controller1(joystickMode::joystickmode_split_arcade);

// Right drive motors
WhoopMotor r1(PORT1, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor r2(PORT2, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor r3(PORT3, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor r4(PORT4, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotorGroup left({&r1, &r2, &r3, &r4});

// Left drive motors
WhoopMotor l1(PORT12, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor l2(PORT13, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor l3(PORT14, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor l4(PORT15, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotorGroup right({&l1, &l2, &l3, &l4});

controller Controller1(controllerType::primary);

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void test_received(std::string message){
  Brain.Screen.clearLine(3);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Pose: %s", message.c_str());
}

void pre_auton(void) {
  pose_messenger.on_message(test_received);

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  manager.start();

  while(true){
    Brain.Screen.clearLine(4);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Pose (e): %s", pose_messenger.read().c_str());
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
