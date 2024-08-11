/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopAutonSelector.hpp                                    */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Autonomous Selector Using Controller                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_AUTON_SELECTOR_HPP
#define WHOOP_AUTON_SELECTOR_HPP

#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/includer.hpp"

namespace whoop {

struct autonomousRoutine {
  std::string auton_name;
  std::function<void()> callback;
  autonomousRoutine(std::string auton_name, std::function<void()> callback)
      : auton_name(auton_name), callback(callback) {}
};

class WhoopAutonSelector : public ComputeNode {
private:
  WhoopController *whoop_controller;
  std::vector<autonomousRoutine> routines;

public:
  WhoopAutonSelector(WhoopController *whoop_controller, std::vector<autonomousRoutine> routines)
      : whoop_controller(whoop_controller), routines(routines) {}
};

} // namespace whoop

#endif // WHOOP_AUTON_SELECTOR_HPP