#ifndef NODE_MANAGER_HPP
#define NODE_MANAGER_HPP

#include "vex.h"
#include <vector>

class ComputeNode;  // Forward declaration

class ComputeManager {
public:
    vex::mutex thread_lock;  // Mutex for synchronization
    std::vector<ComputeNode*> computes;  // Vector to store pointers to compute nodes
    bool debug_mode;

    ComputeManager(bool debugMode=false);
    ComputeManager(std::vector<ComputeNode*> nodes, bool debugMode=false);
    void add_compute_node(ComputeNode* node);
    void start();
};

class ComputeNode {
public:
    vex::mutex* lock_ptr = nullptr;  // Pointer to a mutex for synchronization
    bool running = false;  // Control flag for the task
    bool node_debug = false;
    
    ComputeNode();  // Constructor
    void start_pipeline(bool debug_mode=false);  // Starts the computation process using internal mutex pointer
    void stop_pipeline();  // Stops the computation process
protected:
    virtual void __step();  // Protected helper function for processing steps
    static int task_runner(void* param);  // Static task runner function for PROS
};

#endif // NODE_MANAGER_H
