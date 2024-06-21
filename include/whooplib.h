
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

// Nodes
#include "whooplib/include/toolbox.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"

// Devices
#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/devices/WhoopInertial.hpp"
#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/devices/WhoopDrivetrain.hpp"
#include "whooplib/include/devices/WhoopVision.hpp"

#endif // WHOOP_LIB_H