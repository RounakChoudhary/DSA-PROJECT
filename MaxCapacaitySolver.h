#ifndef MAX_CAPACITY_SOLVER_H
#define MAX_CAPACITY_SOLVER_H

#include <vector>
#include "Network.h"
struct IterationData {
    int iteration;
    std::vector<int> path;
    int flow;
    int bottleneck;
};

class MaxCapacitySolver {
private:
    Network* network;
    std::vector<int> path_result;
    int max_capacity;
    
    // Iteration tracking
    int current_iteration;
    int source_node;
    int target_node;
    std::vector<IterationData> iteration_history;
    
    // Data transfer tracking
    int data_requested;
    int data_remaining;
    int data_transferred;
};
