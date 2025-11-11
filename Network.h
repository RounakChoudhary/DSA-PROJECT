#ifndef NETWORK_H
#define NETWORK_H

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
};
#endif
