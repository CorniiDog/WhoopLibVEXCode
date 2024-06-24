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
BufferNode buffer_system(
  256, // The buffer size, in characters. Increase if necessary, but at the cost of computational efficiency.
  debugMode::debug_disabled, // debugMode::debug_disabled for competition use, debugMode::debug_enabled to allow the code to pass errors through
  "/dev/serial1" // The serial connection of the Jetson Nano ("/dev/serial1" is the micro-usb serial connection on the V5 Brain)
); 

WhoopController controller1(joystickMode::joystickmode_split_arcade);

// Left drive motors
WhoopMotor l1(PORT12, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor l2(PORT13, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor l3(PORT14, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotor l4(PORT15, gearSetting::ratio6_1, reversed::no_reverse);
WhoopMotorGroup left_motors({&l1, &l2, &l3, &l4});

// Right drive motors
WhoopMotor r1(PORT1, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor r2(PORT2, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor r3(PORT3, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotor r4(PORT4, gearSetting::ratio6_1, reversed::yes_reverse);
WhoopMotorGroup right_motors({&r1, &r2, &r3, &r4});

// Vision Offset of the Vision Tesseract from the Center of Robot
RobotVisionOffset vision_offset(
  0.00, // The x offset (+x is the direction of right from the center of the robot in meters).
  15.0/100.0 // The y offset (+y is the direction of forwardness from the center of the robot in meters).
);

// Jetson Nano pose retreival object (also configured on Nano-side) 
WhoopVision vision_system(
  &vision_offset, // pointer to the vision offset
  &buffer_system, // Pointer to the buffer system (will be managed by the buffer system)
  "P" // The subscribed stream name to receive the pose from the Jetson Nano
);

WhoopInertial inertial_sensor(PORT7);
WhoopRotation forward_tracker(PORT6, reversed::yes_reverse);
WhoopRotation sideways_tracker(PORT9, reversed::no_reverse);

// The odom unit center is the virtual intercept of the perpendicular faces of the odometry trackers.
// The measurement is from the center of the odom unit to the designated tracker distances.
// Visual Representation of Tracker Distances from Odom Unit: https://imgur.com/rWCCCfz
WhoopDriveOdomUnit odom_unit(
  to_meters(1.51), // The forward tracker distance, in meters, from the odom unit's center. (positive implies a shift to the right from the odom unit's center)
  to_meters(2.5189), // Diameter of the forward tracker, in meters (e.g., 0.08255 for 3.25-inch wheels).
  to_meters(-4.468), // The sideways tracker distance, in meters, from the odom unit's center (positive implies a shift forward from the odom unit center).
  to_meters(2.5189), // Diameter of the sideways tracker, in meters (e.g., 0.08255 for 3.25-inch wheels).
  &inertial_sensor, // Pointer to the WhoopInertial sensor
  &forward_tracker, // Pointer to the forward tracker, as a WhoopRotation sensor
  &sideways_tracker // Pointer to the sideways tracker, as a WhoopRotation sensor
);

// If your Odom Unit's Center is NOT the center of the robot, apply the offset here.
// The measurement is from the center of the robot to the odom unit center.
// If your Odom Unit's Center IS the center of the robot, set to 0,0.
// Visual representation of Odom Unit from Center of Robot: https://imgur.com/x8ObCIG
WhoopDriveOdomOffset odom_offset(
  &odom_unit, // Pointer to the odometry unit (will manage the odom unit)
  to_meters(-0.6), // The x offset of the odom unit from the center of the robot (positive implies a shift right from the center of the robot).
  to_meters(4.95) // The y offset of the odom unit from the center of the robot (positive implies a shift forward from the center of the robot).
);

WhoopDrivetrain robot_drivetrain(
  &controller1, // Pointer to the controller that controls the drivetrain
  &left_motors, // Pointer to the left motor group (optionally can be a list of motors as well)
  &right_motors // Pointer to the right motor group (optionally can be a list of motors as well)
);

ComputeManager manager({&buffer_system, &robot_drivetrain, &odom_offset});


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

  odom_offset.calibrate();

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
  odom_offset.tare(0,0,0);

  // User control code here, inside the loop
  while (1) {


    Brain.Screen.clearLine(2);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Pose: %.3f %.3f %.3f %.3f %.3f %.3f", vision_system.pose.x, vision_system.pose.y, vision_system.pose.z, vision_system.pose.pitch, vision_system.pose.yaw, vision_system.pose.roll);
    
    
    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Inertial: %.3f", inertial_sensor.get_yaw_radians());

    Brain.Screen.clearLine(4);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Wheel Odom: %.3f %.3f %.3f", to_inches(odom_offset.pose.x), to_inches(odom_offset.pose.y), to_deg(odom_offset.pose.yaw));
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
