/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       whooplib.h                                                */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Primary Headers for WhoopLib                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_LIB_H
#define WHOOP_LIB_H

#ifndef VEX_H
#define VEX_H
#include "vex.h"
#endif // VEX_H

// Toolbox
#include "whooplib/include/toolbox.hpp"

// Calculators
#include "whooplib/include/calculators/TwoDPose.hpp"
#include "whooplib/include/calculators/WheelOdom.hpp"
#include "whooplib/include/calculators/RollingAverage.hpp"
#include "whooplib/include/calculators/LowPassFilter.hpp"
#include "whooplib/include/calculators/KalmanFilter.hpp"
#include "whooplib/include/calculators/PurePursuit.hpp"

// Nodes
#include "whooplib/include/toolbox.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/JetsonCommanderNode.hpp"

// Devices
#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/devices/WhoopInertial.hpp"
#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/devices/WhoopDrivetrain.hpp"
#include "whooplib/include/devices/WhoopVision.hpp"
#include "whooplib/include/devices/WhoopRotation.hpp"
#include "whooplib/include/devices/WhoopDriveOdomUnit.hpp"
#include "whooplib/include/devices/WhoopDriveOdomOffset.hpp"
#include "whooplib/include/devices/WhoopOdomFusion.hpp"

#endif // WHOOP_LIB_H