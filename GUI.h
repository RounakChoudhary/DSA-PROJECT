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
};
#endif
