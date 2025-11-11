#ifndef NETWORK_H
#define NETWORK_H
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

const int INF = std::numeric_limits<int>::max();
struct Edge {
    int to;
    int capacity;
    int flow;
    
    Edge(int t, int cap) : to(t), capacity(cap), flow(0) {}
    
    int residual() const { return capacity - flow; }
};

struct Node {
    float x, y;
    int load;
    bool isActive;
    
    Node(float px = 0, float py = 0) : x(px), y(py), load(0), isActive(true) {}
};

class Network {
private:
    std::vector<Node> nodes;
    std::vector<std::vector<Edge>> adjList;
    int numNodes;
    
public:
    Network(int n) : numNodes(n) {
        nodes.resize(n);
        adjList.resize(n);
    }
    
    void setNodePosition(int id, float x, float y) {
        if (id >= 0 && id < numNodes) {
            nodes[id].x = x;
            nodes[id].y = y;
        }
    }
    void addEdge(int from, int to, int capacity) {
            if (from >= 0 && from < numNodes && to >= 0 && to < numNodes) {
                adjList[from].push_back(Edge(to, capacity));
                adjList[to].push_back(Edge(from, 0)); // Reverse edge for flow algorithms
            }
        }
    void addUndirectedEdge(int from, int to, int capacity) {
        if (from >= 0 && from < numNodes && to >= 0 && to < numNodes) {
            adjList[from].push_back(Edge(to, capacity));
            adjList[to].push_back(Edge(from, capacity));
        }
    }
    
    bool bfs(int source, int sink, std::vector<int>& parent) {
        std::vector<bool> visited(numNodes, false);
        std::queue<int> q;
        q.push(source);
        visited[source] = true;
        parent[source] = -1;
        
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            
            for (size_t i = 0; i < adjList[u].size(); i++) 
            {
                Edge& e = adjList[u][i];
                if (!visited[e.to] && e.residual() > 0) {
                    visited[e.to] = true;
                    parent[e.to] = u;
                    if (e.to == sink) return true;
                    q.push(e.to);
                }
            }
        }
        return false;
    }
//
int maxFlow(int source, int sink) {
        int totalFlow = 0;
        std::vector<int> parent(numNodes);
        
        while (bfs(source, sink, parent)) {
            int pathFlow = INF;
            
            // Find minimum residual capacity along the path
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                for (auto& e : adjList[u]) {
                    if (e.to == v) {
                        pathFlow = std::min(pathFlow, e.residual());
                        break;
                    }
                }
            }
            
            // Update flows
            for (int v = sink; v != source; v = parent[v]) {
                int u = parent[v];
                for (auto& e : adjList[u]) {
                    if (e.to == v) {
                        e.flow += pathFlow;
                        break;
                    }
                }
                for (auto& e : adjList[v]) {
                    if (e.to == u) {
                        e.flow -= pathFlow;
                        break;
                    }
                }
            }
            
            totalFlow += pathFlow;
        }
        
        return totalFlow;
    }
};
#endif
