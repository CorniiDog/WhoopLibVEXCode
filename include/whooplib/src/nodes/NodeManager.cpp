/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       NodeManager.cpp                                           */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Manages the Nodes (Anything that extends ComputeNode)     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/toolbox.hpp"
#include <iostream>
#include <cstdio>

////////////////////////////////////////////////////////////////////////////////
// Node Class Manager
////////////////////////////////////////////////////////////////////////////////

// ComputeManager Methods
ComputeManager::ComputeManager(bool debugMode) : debug_mode(debugMode) {}

// ComputeManager Methods
ComputeManager::ComputeManager(std::vector<ComputeNode*> nodes, bool debugMode): debug_mode(debugMode) {
    for (size_t i = 0; i < nodes.size(); ++i) {
        ComputeNode* node = nodes[i];
        add_compute_node(node);
    }
}

void ComputeManager::add_compute_node(ComputeNode* node) {
    computes.push_back(node);
    node->lock_ptr = &thread_lock;  // Assign the manager's mutex to the node
}

void ComputeManager::start() {
    for (auto& compute : computes) {
        compute->start_pipeline(debug_mode);
    }

}

////////////////////////////////////////////////////////////////////////////////
// Compute Node Base
////////////////////////////////////////////////////////////////////////////////

// ComputeNode Methods
ComputeNode::ComputeNode() {}

int ComputeNode::task_runner(void* param) {
    auto* node = static_cast<ComputeNode*>(param);

    while (node->running) {
        if(node->node_debug){
            try{
                node->__step();
            }
            catch (const std::exception& e) {
                Brain.Screen.clearLine(1);
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("Error: %s", e.what());
            }
        }
        else{
            node->__step();
        }
        vex::wait(20, vex::timeUnits::msec);
    }
    return 1;
}

void ComputeNode::start_pipeline(bool debug_mode) {
    node_debug = debug_mode;
    running = true;
    
    vex::task myTask(ComputeNode::task_runner, this);
}

void ComputeNode::stop_pipeline() {
    running = false;
}

void ComputeNode::__step() {
    if (lock_ptr) {
        lock_ptr->lock();  // Acquire the mutex
        // Simulate some computations or operations here
        lock_ptr->unlock();  // Release the mutex
    }
}
