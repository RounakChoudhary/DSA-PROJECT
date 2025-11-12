#ifndef CONGESTION_MANAGER_H
#define CONGESTION_MANAGER_H

#include "Network.h"
#include <vector>
#include <cmath>

struct TrafficRequest {
    int source;
    int destination;
    int demand;
    bool routed;
    std::vector<int> path;
    
    TrafficRequest(int s, int d, int dem) 
        : source(s), destination(d), demand(dem), routed(false) {}
};
#endif
