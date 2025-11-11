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
