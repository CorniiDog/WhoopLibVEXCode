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

//TODO: Merge the wheel odom pose and vision pose
// Use inertial for pitch, yaw, roll
// Use wheel odom for x, y, yaw
// Use vision system for x, y, z, pitch, yaw, roll, confidence, which confidence is [0,1] and conf>0.5 is good, conf>0.3 is acceptable

////////////////////////////////////////////////////////////
/**
 *    Globals
 */
////////////////////////////////////////////////////////////

// Primary controller
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

// Sensors
WhoopInertial inertial_sensor(PORT7);
WhoopRotation forward_tracker(PORT6, reversed::yes_reverse);
WhoopRotation sideways_tracker(PORT9, reversed::no_reverse);

////////////////////////////////////////////////////////////
/**
 *    Wheel Odometry Configuration
 */
////////////////////////////////////////////////////////////

// The odom unit center is the virtual intercept of the perpendicular faces of the odometry trackers.
// The measurement is from the center of the odom unit to the designated tracker distances.
// Visual Representation of Tracker Distances from Odom Unit: https://imgur.com/rWCCCfz

WhoopDriveOdomUnit odom_unit(
  to_meters(1.51), // The forward tracker distance, in meters, from the odom unit's center. (positive implies a shift to the right from the odom unit's center)
  to_meters(2.5189), // Diameter of the forward tracker, in meters (e.g., 0.08255 for 3.25-inch wheels).
  to_meters(-4.468), // The sideways tracker distance, in meters, from the odom unit's center (positive implies a shift forward from the odom unit center)
  to_meters(2.5189), // Diameter of the sideways tracker, in meters (e.g., 0.08255 for 3.25-inch wheels).
  &inertial_sensor, // Pointer to the WhoopInertial sensor
  &forward_tracker, // Pointer to the forward tracker, as a WhoopRotation sensor
  &sideways_tracker // Pointer to the sideways tracker, as a WhoopRotation sensor
);


// Example of a single tracker setup
// In this configuration, the sideways tracker must be directly in front, behind, or on the center of the robot's rotation.
// It cannot be right or left shifted from the center. This is a similar configuration to the JAR-Template's one-tracker odom.
// After, configure the odom_offset to 0,0
/*
WhoopDriveOdomUnit odom_unit(
  to_meters(12.625), // Width of the drivetrain, in meters. Measured as the distance between the left wheels and right wheels
  to_meters(3), // Diameter of drivetrain wheels, in meters 
  1.0/2.0, // Gear Ratio of Drivetrain (If [motor is powering 32t] connected to [64t sharing shaft with drive wheel], it would be ratio = 32/64 = 1.0/2.0) 
  to_meters(0), // Sideways tracker distance from the center of the robot's rotation. (positive implies a shift forward from the drivetrain's center)
  to_meters(2.5189), // Diameter of the sideways tracker, in meters (e.g., 0.08255 for 3.25-inch wheels).
  &inertial_sensor, 
  &sideways_tracker, 
  &left_motors, 
  &right_motors
);
*/

// Example of a no-tracker setup
// In this configuration, there is not much you really need to do.
// After, configure the odom_offset below to 0,0
/*
WhoopDriveOdomUnit odom_unit(
  to_meters(12.625), // Width of the drivetrain, in meters. Measured as the distance between the left wheels and right wheels
  to_meters(3), // Diameter of drivetrain wheels, in meters 
  1.0/2.0, // Gear Ratio of Drivetrain (If [motor is powering 32t] connected to [64t sharing shaft with drive wheel], it would be ratio = 32/64 = 1.0/2.0) 
  &inertial_sensor, 
  &left_motors, 
  &right_motors
);
*/

// If your Odom Unit's Center is NOT the center of the robot, apply the offset here.
// The measurement is from the center of the robot to the odom unit center.
// If your Odom Unit's Center IS the center of the robot, set to 0,0.
// ALSO If using one tracker or no tracker, set to 0,0.
// Visual representation of Odom Unit from Center of Robot: https://imgur.com/x8ObCIG
WhoopDriveOdomOffset odom_offset(
  &odom_unit, // Pointer to the odometry unit (will manage the odom unit)
  to_meters(-0.6), // The x offset of the odom unit from the center of the robot (positive implies a shift right from the center of the robot).
  to_meters(4.95) // The y offset of the odom unit from the center of the robot (positive implies a shift forward from the center of the robot).
);

////////////////////////////////////////////////////////////
/**
 *    VISION TESSERACT
 */
////////////////////////////////////////////////////////////

// If you have a Vision Tesseract on your robot, 
// switch to jetsonCommunication::enable_comms
// If not, use jetsonCommunication::disable_comms
jetsonCommunication jetson_comms_enabled = jetsonCommunication::enable_comms;

////////////////////////////////////////////////////////////
/** 
 *    Skip the rest of this section if you don't have a
 *    Tesseract on your robot
*/ 
////////////////////////////////////////////////////////////

// Serial communication module
BufferNode buffer_system(
  256, // The buffer size, in characters. Increase if necessary, but at the cost of computational efficiency.
  debugMode::debug_disabled, // debugMode::debug_disabled for competition use, debugMode::debug_enabled to allow the code to pass errors through
  "/dev/serial1" // The serial connection of the Jetson Nano ("/dev/serial1" is the micro-usb serial connection on the V5 Brain, "/dev/serial2" is controller)
); 


// Vision Offset of the Vision Tesseract from the Center of Robot
RobotVisionOffset vision_offset(
  0.00, // The x offset in meters, (right-positive from the center of the robot).
  240.0/1000.0 // The y offset in meters (forward-positive from the center of the robot).
);

// Jetson Nano pose retreival object (also configured on Nano-side) 
WhoopVision vision_system(
  &vision_offset, // pointer to the vision offset
  &buffer_system, // Pointer to the buffer system (will be managed by the buffer system)
  "P" // The subscribed stream name to receive the pose from the Jetson Nano
);

// This is the jetson commander. It sends keep-alive messages intermittently and also allows
// Running the following functions (can be a touch screen confirmation button perhaps):
// jetson_commander.shutdown_jetson();
// jetson_commander.reboot_jetson();
// jetson_commander.restart_vision_process();
// bool is_connected_currently = jetson_commander.is_connected_to_jetson();
// This is essential to ensure that the nano starts its internal program, stop program, restarts program, 
// and can be told to reboot or shutdown
JetsonCommander jetson_commander(
  &controller1, // The controller to send messages to upon error
  &buffer_system, // Pointer to the buffer system (will be managed by the buffer system)
  "C", // The subscribed stream name for keep-alive, shutdown, and reboot
  60, // In seconds. When the V5 Brain shuts down or disconnects, the Jetson Nano will keep the program running for this time before it shuts off
  2, // How many seconds to wait before sending anoter keep alive message to Jetson (suggested 2)
  jetsonCommunication::enable_comms // If you don't have a Vision Tesseract on your robot, set to disable_comms
);

////////////////////////////////////////////////////////////
/**
 *    Vision x Wheel Odometry Fusion
 */
////////////////////////////////////////////////////////////
WhoopOdomFusion odom_fusion(
  &vision_system, // Pointer to the vision system
  &odom_offset, // Pointer to the odometry offset
  0.9, // Minimum confidence threshold to apply vision system to odometry
  FusionMode::wheel_odom_only, // The method of fusing
  to_meters(50), // If FusionMode is fusion_gradual, it is the maximum allowable lateral shift the vision camera can update in meters per second.
  to_rad(500) // If FusionMode is fusion_gradual, it is the maximum allowable yaw rotational shift the vision camera can update in radians per second.
);

WhoopOdomCommunicator odom_communicator(
  &buffer_system, // Pointer to the buffer system to communicate the messenger to
  &vision_offset, // Pointer to the vision offset configured above
  &odom_offset, // Pointer to the drive offset object
  "O", // The string that represents the odometry stream to send over 
  4, // The number of decimal places of the pose data (measurements in meters/radians). Higher decimal places is better precision, but larger serial packets
  3 // The number of elements for rolling average (recommended 3) to smoothen velocity
);

////////////////////////////////////////////////////////////
/**
 *    Robot Drivetrain and Manager
 */
////////////////////////////////////////////////////////////
WhoopDrivetrain robot_drivetrain(
  &odom_fusion, // Odometry fusion module
  PoseUnits::in_deg_cw, // Set default pose units if not defined. "m_deg_cw" means "meters, degrees, clockwise-positive yaw", "in_deg_ccw" means "inches, degrees, counter-clockwise-positive yaw", and so forth.
  &controller1, // Pointer to the controller 
  &left_motors, // Pointer to the left motor group (optionally can be a list of motors as well)
  &right_motors // Pointer to the right motor group (optionally can be a list of motors as well)
);

ComputeManager manager({&buffer_system, &jetson_commander, &robot_drivetrain, &controller1, &odom_communicator});

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
  controller1.notify("Initializing", 2);
  wait(0.5, sec);
  jetson_commander.initialize();
  robot_drivetrain.calibrate();

  wait(5, sec); // Wait 2 seconds to allow the vision system to initialize to 0
  robot_drivetrain.set_pose_units(PoseUnits::in_deg_cw); // Inches, degrees, clockwise-positive
  robot_drivetrain.set_pose(0,0,0);

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
    Brain.Screen.clearLine(1);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Approving Frames: %s", boolToString(odom_fusion.approving_frames()).c_str());

    Brain.Screen.clearLine(2);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Vision Running: %s Approving Frames: %s", boolToString(vision_system.vision_running()).c_str());

    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Jetson Connected: %s", boolToString(jetson_commander.is_connected_to_jetson()).c_str());

    // Fusion Odometry
    Pose current_pose = robot_drivetrain.get_pose();
    Brain.Screen.clearLine(4);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("FO (%s): %.1f %.1f %.1f %.1f %.1f %.1f", robot_drivetrain.get_units_str().c_str(), current_pose.x, current_pose.y, current_pose.z, current_pose.pitch, current_pose.yaw, current_pose.roll);

    // Vision Odometry
    Pose vision_pose = vision_system.get_pose();
    Brain.Screen.clearLine(5);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("VO (m_rad_ccw): %.2f %.2f %.2f %.2f %.2f %.2f", vision_pose.x, vision_pose.y, vision_pose.z, vision_pose.pitch, vision_pose.yaw, vision_pose.roll);

    // Wheel Odometry
    TwoDPose wheel_pose = odom_offset.get_pose(); // raw odometry (untranslated) readings
    Brain.Screen.clearLine(6);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("WO (m_rad_ccw): %.2f %.2f %.2f", wheel_pose.x, wheel_pose.y, wheel_pose.yaw);
    

    // Object-Space Velocity
    TwoDPose obj_space_velocity = odom_communicator.relative_velocity;
    Brain.Screen.clearLine(7);
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("Velocities (m/s_rad/s_ccw): %.2f %.2f %.2f", obj_space_velocity.x, obj_space_velocity.y, obj_space_velocity.yaw);

    wait(100, msec);
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
