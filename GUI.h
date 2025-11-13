#ifndef GUI_H
#define GUI_H

#include "raylib.h"
#include "Network.h"
#include "MaxCapacitySolver.h"
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>

enum AppState {
    MENU,
    SELECT_NODES,
    VISUALIZE_ITERATION,
    SUMMARY
};

class GUI {
private:
    Network* network;
    MaxCapacitySolver* solver;
    AppState currentState;
    
    // Node selection
    int source_node;
    int target_node;
    bool source_selected;
    bool target_selected;
    
    // Path visualization
    std::vector<int> highlighted_path;
    
    // Data transfer input
    std::string data_transfer_text;
    bool data_transfer_input_active;
    int data_transfer_amount;
    
    // Helper: Calculate node positions in circular layout
    void calculateNodePositions() {
        if (!network) return;
        int n = network->getNumNodes();
        if (n == 0) return;
        
        int width = 800;
        int height = 600;
        int centerX = width / 2;
        int centerY = height / 2;
        float radius = std::min(width, height) * 0.35f;
        
        for (int i = 0; i < n; i++) {
            float angle = 2.0f * PI * i / n - PI / 2.0f;
            float x = centerX + radius * cosf(angle);
            float y = centerY + radius * sinf(angle);
            network->setNodePosition(i, x, y);
        }
    }
// Helper: Check if point is in circle
    bool isPointInCircle(Vector2 point, Vector2 center, float radius) {
        float dx = point.x - center.x;
        float dy = point.y - center.y;
        return (dx * dx + dy * dy) <= radius * radius;
    }
    
    // Helper: Get node at mouse position
    int getNodeAtPosition(Vector2 mousePos) {
        if (!network) return -1;
        for (int i = 0; i < network->getNumNodes(); i++) {
            const Node& node = network->getNode(i);
            Vector2 nodePos = {node.x, node.y};
            if (isPointInCircle(mousePos, nodePos, 20.0f)) {
                return i;
            }
        }
        return -1;
    }
    
    // Draw the graph
    void DrawGraph(bool highlightPath = false) {
        if (!network) return;
        
        // Draw edges
        for (int i = 0; i < network->getNumNodes(); i++) {
            const Node& fromNode = network->getNode(i);
            const auto& edges = network->getEdges(i);
            
            for (const auto& e : edges) {
                // Only draw each edge once (avoid duplicates in undirected graph)
                if (e.to <= i) continue;
                
                const Node& toNode = network->getNode(e.to);
                
                // Check if this edge is on the highlighted path
                bool isPathEdge = false;
                if (highlightPath && !highlighted_path.empty()) {
                    for (size_t j = 0; j < highlighted_path.size() - 1; j++) {
                        if ((highlighted_path[j] == i && highlighted_path[j + 1] == e.to) ||
                            (highlighted_path[j] == e.to && highlighted_path[j + 1] == i)) {
                            isPathEdge = true;
                            break;
                        }
                    }
                }
                
                // Draw edge
                Color edgeColor = isPathEdge ? GOLD : DARKGRAY;
                float thickness = isPathEdge ? 4.0f : 2.0f;
                DrawLineEx({fromNode.x, fromNode.y}, {toNode.x, toNode.y}, thickness, edgeColor);
                
                // Draw capacity label
                if (e.capacity > 0) {
                    float midX = (fromNode.x + toNode.x) / 2.0f;
                    float midY = (fromNode.y + toNode.y) / 2.0f;
                    std::string capStr = std::to_string(e.capacity);
                    Vector2 textSize = MeasureTextEx(GetFontDefault(), capStr.c_str(), 12, 1.0f);
                    DrawRectangle(midX - textSize.x/2 - 3, midY - textSize.y/2 - 2, 
                                 textSize.x + 6, textSize.y + 4, WHITE);
                    DrawText(capStr.c_str(), midX - textSize.x/2, midY - textSize.y/2, 12, BLACK);
                }
            }
        }
        
        // Draw nodes
        for (int i = 0; i < network->getNumNodes(); i++) {
            const Node& node = network->getNode(i);
            Color nodeColor = WHITE;
            
            // Highlight source and target nodes
            if (i == source_node && source_selected) {
                nodeColor = GREEN;
            } else if (i == target_node && target_selected) {
                nodeColor = RED;
            }
            
            // Highlight nodes on path
            if (highlightPath && !highlighted_path.empty()) {
                if (std::find(highlighted_path.begin(), highlighted_path.end(), i) != highlighted_path.end()) {
                    nodeColor = GOLD;
                }
            }
            
            DrawCircle(node.x, node.y, 20, nodeColor);
            DrawCircleLines(node.x, node.y, 20, BLACK);
            
            // Draw node label
            std::string label = std::to_string(i);
            Vector2 textSize = MeasureTextEx(GetFontDefault(), label.c_str(), 16, 1.0f);
            DrawText(label.c_str(), node.x - textSize.x/2, node.y - textSize.y/2, 16, BLACK);
        }
    }
};
#endif
