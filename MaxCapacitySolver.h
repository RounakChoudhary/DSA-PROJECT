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
 void dijkstraMaxMin(int source, int target) {
        int n = network->getNumNodes();
        std::vector<int> max_cap(n, -1); // Maximum minimum capacity to reach each node
        std::vector<int> parent(n, -1);
        std::vector<bool> visited(n, false);
        
        // Priority queue: {max_min_capacity, node}
        // We use negative capacity to make it a max-heap (since priority_queue is min-heap by default)
        std::priority_queue<std::pair<int, int>> pq;
        
        max_cap[source] = std::numeric_limits<int>::max();
        pq.push({max_cap[source], source});
        
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
                // The capacity to reach v via u is min(max_cap[u], e.capacity)
                int new_cap = std::min(max_cap[u], e.capacity);
                
                if (new_cap > max_cap[v]) {
                    max_cap[v] = new_cap;
                    parent[v] = u;
                    pq.push({max_cap[v], v});
                }
            }
        }
        
        // Reconstruct path
        path_result.clear();
        if (max_cap[target] >= 0) {
            int current = target;
            while (current != -1) {
                path_result.push_back(current);
                current = parent[current];
            }
            std::reverse(path_result.begin(), path_result.end());
            max_capacity = max_cap[target];
        } else {
            max_capacity = -1; // No path found
        }
    }
public:
    MaxCapacitySolver(Network* net) : network(net), max_capacity(-1), 
                                      current_iteration(0), source_node(-1), target_node(-1),
                                      data_requested(0), data_remaining(0), data_transferred(0) {}
    
    void solve(int source, int target) {
        path_result.clear();
        if (source >= 0 && source < network->getNumNodes() && 
            target >= 0 && target < network->getNumNodes() && 
            source != target) {
            dijkstraMaxMin(source, target);
        }
    }
void initializeIterations(int source, int target, int data_amount = 0) {
        source_node = source;
        target_node = target;
        current_iteration = 0;
        iteration_history.clear();
        path_result.clear();
        max_capacity = -1;
        data_requested = data_amount;
        data_remaining = data_amount;
        data_transferred = 0;
    }
    
    // Run one iteration step
    bool runStep() {
        if (source_node < 0 || target_node < 0) {
            return false;
        }
        
        // If data transfer amount is set and all data is transferred, stop
        if (data_requested > 0 && data_remaining <= 0) {
            return false; // All requested data has been transferred
        }
        
        std::vector<int> path;
        int bottleneck;
        
        if (!findCheapestPath(source_node, target_node, path, bottleneck)) {
            return false; // No path found
        }
        
        // Check if bottleneck is positive (path exists but has no capacity)
        if (bottleneck <= 0) {
            return false; // Path exists but no capacity available
        }
        
        // Calculate flow: limit to remaining data if data transfer amount is set
        int flow = bottleneck;
        if (data_requested > 0) {
            flow = std::min(bottleneck, data_remaining);
        }
        
        // Update capacities along the path (reduce by flow)
        for (size_t i = 0; i < path.size() - 1; i++) {
            int u = path[i];
            int v = path[i + 1];
            
            // Get modifiable edges
            auto& edges = network->getEdgesModifiable(u);
            for (auto& e : edges) {
                if (e.to == v) {
                    e.capacity -= flow;
                    if (e.capacity < 0) e.capacity = 0;
                    break;
                }
            }
        }
        
        current_iteration++;
        
        // Update data tracking
        if (data_requested > 0) {
            data_transferred += flow;
            data_remaining -= flow;
        }
};
