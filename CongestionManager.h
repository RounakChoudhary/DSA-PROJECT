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
class CongestionManager {
private:
    Network* network;
    std::vector<TrafficRequest> requests;
    float congestionLevel;
    int totalCapacity;
    int totalFlow;
    
    bool findPath(int source, int sink, std::vector<int>& path, std::vector<bool>& visited) {
        if (source == sink) {
            path.push_back(sink);
            return true;
        }
        
        visited[source] = true;
        const auto& edges = network->getEdges(source);
        
        for (const auto& e : edges) {
            if (!visited[e.to] && e.residual() > 0) {
                if (findPath(e.to, sink, path, visited)) {
                    path.insert(path.begin(), source);
                    return true;
                }
            }
        }
        
        return false;
    }
public:
    CongestionManager(Network* net) 
        : network(net), congestionLevel(0.0f), totalCapacity(0), totalFlow(0) {}
    
    void addRequest(int source, int dest, int demand) {
        requests.emplace_back(source, dest, demand);
    }
    
    void routeTraffic() {
        network->resetFlows();
        
        for (auto& req : requests) {
            // Try to route each request
            int flow = network->maxFlow(req.source, req.destination);
            if (flow >= req.demand) {
                req.routed = true;
                
                // Find the path taken
                std::vector<int> path;
                std::vector<bool> visited(network->getNumNodes(), false);
                findPath(req.source, req.destination, path, visited);
                req.path = path;
            } else {
                req.routed = false;
            }
        }
        
        network->updateNodeLoad();
        calculateCongestion();
    }
}
#endif
