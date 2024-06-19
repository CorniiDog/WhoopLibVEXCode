
#ifndef WHOOP_LIB_H
#define WHOOP_LIB_H

#ifndef VEX_H
#define VEX_H
#include "vex.h"
#endif

// Toolbox
#include "whooplib/include/toolbox.hpp"

// Calculators
#include "whooplib/include/calculators/TwoDPose.hpp"

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

#endif