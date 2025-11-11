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

    bool findCheapestPath(int source, int target, std::vector<int>& path, int& bottleneck) {
        int n = network->getNumNodes();
        std::vector<double> dist(n, std::numeric_limits<double>::max());
        std::vector<int> parent(n, -1);
        std::vector<bool> visited(n, false);
        
        std::priority_queue<std::pair<double, int>, 
                           std::vector<std::pair<double, int>>,
                           std::greater<std::pair<double, int>>> pq;
        
        dist[source] = 0.0;
        pq.push({0.0, source});
        
        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            
            if (visited[u]) continue;
            visited[u] = true;
            
            if (u == target) break;
            
            const auto& edges = network->getEdges(u);
            for (const auto& e : edges) {
                // Only consider edges with positive capacity
                if (e.capacity <= 0) continue;
                
                int v = e.to;
                // Weight calculation for path selection (prefers higher capacity)
                double edgeWeight = 1.0 / (0.1 + static_cast<double>(e.capacity));
                
                if (dist[u] + edgeWeight < dist[v]) {
                    dist[v] = dist[u] + edgeWeight;
                    parent[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }
        
        if (parent[target] == -1) {
            return false; // No path found
        }
        
        // Reconstruct path and calculate bottleneck
        path.clear();
        bottleneck = std::numeric_limits<int>::max();
        
        int current = target;
        while (current != -1) {
            path.push_back(current);
            if (parent[current] != -1) {
                // Find edge capacity
                const auto& edges = network->getEdges(parent[current]);
                for (const auto& e : edges) {
                    if (e.to == current) {
                        bottleneck = std::min(bottleneck, e.capacity);
                        break;
                    }
                }
            }
            current = parent[current];
        }
        
        std::reverse(path.begin(), path.end());
        return true;
    }
};
