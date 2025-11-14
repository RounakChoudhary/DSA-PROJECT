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
// Draw Menu screen
    void DrawMenu() {
        DrawText("Max Capacity Network Flow Visualizer", 200, 100, 30, BLACK);
        DrawText("Click the button below to load demo graph", 250, 200, 18, DARKGRAY);
        
        // Demo button
        Rectangle demoButton = {300, 400, 200, 50};
        Color demoButtonColor = CheckCollisionPointRec(GetMousePosition(), demoButton) ? GRAY : GREEN;
        DrawRectangleRec(demoButton, demoButtonColor);
        DrawRectangleLinesEx(demoButton, 2, BLACK);
        
        const char* demoText = "Load Demo Graph";
        Vector2 textSize = MeasureTextEx(GetFontDefault(), demoText, 18, 1.0f);
        float textX = demoButton.x + (demoButton.width - textSize.x) / 2.0f;
        float textY = demoButton.y + (demoButton.height - textSize.y) / 2.0f;
        DrawText(demoText, (int)textX, (int)textY, 18, WHITE);
    }
    
    // Draw Select Nodes screen
    void DrawSelectNodes() {
        DrawText("Select Source and Target Nodes", 200, 30, 25, BLACK);
        
        // Draw graph
        DrawGraph(false);
        
        // Instructions
        std::string instruction;
        if (!source_selected) {
            instruction = "Click on a node to select SOURCE";
        } else if (!target_selected) {
            instruction = "Click on a node to select TARGET (different from source)";
        } else {
            instruction = "Source: " + std::to_string(source_node) + " | Target: " + std::to_string(target_node);
        }
        DrawText(instruction.c_str(), 200, 480, 18, DARKGRAY);
        
        // Data transfer input
        DrawText("Data Transfer Amount (optional):", 200, 510, 18, DARKGRAY);
        Rectangle dataInputBox = {200, 535, 200, 30};
        Color dataInputColor = data_transfer_input_active ? YELLOW : LIGHTGRAY;
        DrawRectangleRec(dataInputBox, dataInputColor);
        DrawRectangleLinesEx(dataInputBox, 2, DARKGRAY);
        DrawText((data_transfer_text.empty() ? "Enter amount..." : data_transfer_text.c_str()), 210, 542, 18, BLACK);
        
        // Run Solver button
        Rectangle button = {600, 540, 150, 40};
        Color buttonColor = (source_selected && target_selected) ? GREEN : GRAY;
        DrawRectangleRec(button, buttonColor);
        DrawRectangleLinesEx(button, 2, BLACK);
        DrawText("Run Solver", 620, 550, 18, BLACK);
        
        DrawText("Press ESC to go back", 50, 550, 16, GRAY);
    }
    
    // Draw Iteration Visualization screen
    void DrawVisualizeIteration() {
        // Header
        DrawRectangle(0, 0, 800, 50, Fade(DARKBLUE, 0.8f));
        DrawRectangleLines(0, 0, 800, 50, DARKBLUE);
        
        // Draw graph with path highlighting
        DrawGraph(true);
        
        // Iteration info panel
        DrawRectangle(10, 440, 780, 150, Fade(WHITE, 0.95f));
        DrawRectangleLines(10, 440, 780, 150, DARKBLUE);
        
        const auto& history = solver->getIterationHistory();
        std::string title = "Iteration View";
        
        if (!history.empty()) {
            // Show the most recent iteration
            int iterIndex = history.size() - 1;
            const auto& iter = history[iterIndex];
            title = "Iteration " + std::to_string(iter.iteration);
            
            int yPos = 450;
            
            // Iteration number
            std::string iterTitle = "Iteration " + std::to_string(iter.iteration);
            DrawRectangle(15, yPos - 2, 200, 28, Fade(DARKBLUE, 0.3f));
            DrawText(iterTitle.c_str(), 20, yPos, 22, DARKBLUE);
            yPos += 32;
            
            // Path
            DrawText("Path Taken:", 20, yPos, 18, DARKGRAY);
            yPos += 22;
            
            for (size_t i = 0; i < iter.path.size(); i++) {
                DrawCircle(25 + i * 80, yPos + 10, 12, Fade(BLUE, 0.3f));
                DrawCircleLines(25 + i * 80, yPos + 10, 12, BLUE);
                DrawText(std::to_string(iter.path[i]).c_str(), 20 + i * 80, yPos + 3, 16, DARKBLUE);
                
                if (i < iter.path.size() - 1) {
                    DrawLineEx({37 + i * 80, yPos + 10}, {63 + i * 80, yPos + 10}, 2, BLUE);
                    DrawTriangle({63 + i * 80, yPos + 10}, {58 + i * 80, yPos + 5}, {58 + i * 80, yPos + 15}, BLUE);
                }
            }
            yPos += 30;
            
            // Flow and bottleneck
            DrawRectangle(20, yPos, 150, 25, Fade(LIGHTGRAY, 0.5f));
            DrawRectangleLines(20, yPos, 150, 25, DARKGRAY);
            DrawText(("Flow: " + std::to_string(iter.flow)).c_str(), 25, yPos + 4, 16, BLACK);
            
            DrawRectangle(180, yPos, 150, 25, Fade(LIGHTGRAY, 0.5f));
            DrawRectangleLines(180, yPos, 150, 25, DARKGRAY);
            DrawText(("Bottleneck: " + std::to_string(iter.bottleneck)).c_str(), 185, yPos + 4, 16, BLACK);
            yPos += 30;
            
            // Data transfer info if set
            if (solver->getDataRequested() > 0) {
                DrawRectangle(20, yPos, 250, 25, Fade(YELLOW, 0.3f));
                DrawRectangleLines(20, yPos, 250, 25, DARKGRAY);
                std::string dataInfo = "Data: " + std::to_string(solver->getDataTransferred()) + 
                                     "/" + std::to_string(solver->getDataRequested()) + 
                                     " (Remaining: " + std::to_string(solver->getDataRemaining()) + ")";
                DrawText(dataInfo.c_str(), 25, yPos + 4, 14, BLACK);
                yPos += 30;
            }
            
            // Total iterations
            DrawText(("Total Iterations: " + std::to_string(solver->getCurrentIteration())).c_str(), 20, yPos, 16, DARKBLUE);
        } else {
            DrawText("No iteration data available", 20, 450, 18, RED);
        }
        
        // Title
        Vector2 titleSize = MeasureTextEx(GetFontDefault(), title.c_str(), 28, 1.0f);
        DrawText(title.c_str(), (int)(400 - titleSize.x/2), 10, 28, WHITE);
        
        // Action buttons - check if more iterations are possible
        // NOTE: This only affects which button is shown, NOT the iteration display
        // The final iteration is ALWAYS shown above, regardless of canContinue() value
        bool canContinue = solver->canContinue();
        
        if (canContinue) {
            // More iterations possible - show Next Step button
            Rectangle nextButton = {600, 520, 150, 50};
            Color nextButtonColor = CheckCollisionPointRec(GetMousePosition(), nextButton) ? Fade(GREEN, 0.7f) : GREEN;
            DrawRectangleRec(nextButton, nextButtonColor);
            DrawRectangleLinesEx(nextButton, 3, DARKGREEN);
            
            const char* nextText = "Next Step";
            Vector2 nextTextSize = MeasureTextEx(GetFontDefault(), nextText, 20, 1.0f);
            DrawText(nextText, (int)(nextButton.x + (nextButton.width - nextTextSize.x) / 2), 
                    (int)(nextButton.y + (nextButton.height - nextTextSize.y) / 2), 20, WHITE);
        } else {
            // Final iteration completed - show completion message and Summary button
            // IMPORTANT: The final iteration is displayed above and remains visible
            // User must explicitly click "Show Summary" to proceed
            
            // Completion message
            DrawRectangle(15, 515, 250, 30, Fade(GREEN, 0.2f));
            DrawRectangleLines(15, 515, 250, 30, GREEN);
            std::string completionMsg;
            if (solver->getDataRequested() > 0 && solver->getDataRemaining() <= 0) {
                completionMsg = "All data transferred!";
            } else {
                completionMsg = "All paths explored!";
            }
            DrawText(completionMsg.c_str(), 20, 520, 18, DARKGREEN);
            
            // Show Summary button - user must click this to proceed
            Rectangle summaryButton = {600, 520, 150, 50};
            Color summaryButtonColor = CheckCollisionPointRec(GetMousePosition(), summaryButton) ? Fade(BLUE, 0.7f) : BLUE;
            DrawRectangleRec(summaryButton, summaryButtonColor);
            DrawRectangleLinesEx(summaryButton, 3, DARKBLUE);
            
            const char* summaryText = "Show Summary";
            Vector2 summaryTextSize = MeasureTextEx(GetFontDefault(), summaryText, 18, 1.0f);
            DrawText(summaryText, (int)(summaryButton.x + (summaryButton.width - summaryTextSize.x) / 2), 
                    (int)(summaryButton.y + (summaryButton.height - summaryTextSize.y) / 2), 18, WHITE);
        }
        
        DrawText("Press ESC to go back", 600, 575, 16, GRAY);
    }
    
    // Draw Summary screen
    void DrawSummary() {
        DrawText("Summary", 350, 30, 28, BLACK);
        
        // Draw graph
        DrawGraph(false);
        
        // Summary information
        int y = 50;
        DrawText("=== Summary ===", 50, y, 22, DARKBLUE);
        y += 30;
        
        const auto& history = solver->getIterationHistory();
        
        if (!history.empty()) {
            // Total iterations
            DrawText(("Total Iterations: " + std::to_string(solver->getCurrentIteration())).c_str(), 50, y, 18, BLACK);
            y += 25;
            
            // Calculate total flow
            int totalFlow = 0;
            for (const auto& iter : history) {
                totalFlow += iter.flow;
            }
            DrawText(("Total Flow: " + std::to_string(totalFlow)).c_str(), 50, y, 18, BLACK);
            y += 30;
            
            // Data transfer summary if set
            if (solver->getDataRequested() > 0) {
                DrawText(("Data Requested: " + std::to_string(solver->getDataRequested())).c_str(), 50, y, 18, BLACK);
                y += 25;
                DrawText(("Data Transferred: " + std::to_string(solver->getDataTransferred())).c_str(), 50, y, 18, BLACK);
                y += 25;
                DrawText(("Data Remaining: " + std::to_string(solver->getDataRemaining())).c_str(), 50, y, 18, BLACK);
                y += 30;
            }
            
            // Iteration details
            DrawText("=== Iteration Details ===", 50, y, 20, DARKBLUE);
            y += 25;
            
            for (const auto& iter : history) {
                std::string iterTitle = "Iteration " + std::to_string(iter.iteration) + ":";
                DrawText(iterTitle.c_str(), 50, y, 16, DARKGRAY);
                y += 20;
                
                // Path
                std::string pathStr = "  Path: ";
                for (size_t i = 0; i < iter.path.size(); i++) {
                    pathStr += std::to_string(iter.path[i]);
                    if (i < iter.path.size() - 1) pathStr += " -> ";
                }
                DrawText(pathStr.c_str(), 50, y, 14, BLACK);
                y += 18;
                
                // Flow and bottleneck
                std::string iterInfo = "  Flow: " + std::to_string(iter.flow) + 
                                     " | Bottleneck: " + std::to_string(iter.bottleneck);
                DrawText(iterInfo.c_str(), 50, y, 14, BLACK);
                y += 20;
            }
        } else {
            DrawText("No iteration data available", 50, y, 18, RED);
        }
        
        // Back button
        Rectangle backButton = {600, 540, 150, 40};
        Color backButtonColor = CheckCollisionPointRec(GetMousePosition(), backButton) ? GRAY : BLUE;
        DrawRectangleRec(backButton, backButtonColor);
        DrawRectangleLinesEx(backButton, 2, BLACK);
        DrawText("Back", 630, 550, 18, WHITE);
    }
    
    // Process Menu input
    void ProcessMenuInput() {
        // Handle Demo button
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Rectangle demoButton = {300, 400, 200, 50};
            Vector2 mousePos = GetMousePosition();
            if (CheckCollisionPointRec(mousePos, demoButton)) {
                // Load demo network
                network = new Network(5);
                setupDemoNetwork(*network);
                calculateNodePositions();
                solver = new MaxCapacitySolver(network);
                currentState = SELECT_NODES;
                source_selected = false;
                target_selected = false;
            }
        }
    }
    
    // Process Select Nodes input
    void ProcessSelectNodes() {
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector2 mousePos = GetMousePosition();
            int nodeId = getNodeAtPosition(mousePos);
            
            // Check data transfer input box
            Rectangle dataInputBox = {200, 535, 200, 30};
            if (CheckCollisionPointRec(mousePos, dataInputBox)) {
                data_transfer_input_active = true;
            } else {
                data_transfer_input_active = false;
            }
            
            if (nodeId >= 0) {
                if (!source_selected) {
                    source_node = nodeId;
                    source_selected = true;
                } else if (!target_selected && nodeId != source_node) {
                    target_node = nodeId;
                    target_selected = true;
                }
            }
            
            // Check Run Solver button
            Rectangle button = {600, 540, 150, 40};
            if (CheckCollisionPointRec(mousePos, button) && source_selected && target_selected) {
                // Initialize iterations with data transfer amount
                int data_amount = (data_transfer_amount > 0) ? data_transfer_amount : 0;
                solver->initializeIterations(source_node, target_node, data_amount);
                
                // Run first iteration
                if (solver->runStep()) {
                    // Update highlighted path to show the first iteration
                    const auto& history = solver->getIterationHistory();
                    if (!history.empty()) {
                        highlighted_path = history[0].path;
                    } else {
                        highlighted_path = solver->getPath();
                    }
                    currentState = VISUALIZE_ITERATION;
                }
            }
        }
        
        // Handle data transfer text input
        if (data_transfer_input_active) {
            int key = GetCharPressed();
            if (key >= '0' && key <= '9') {
                data_transfer_text += (char)key;
            } else if (IsKeyPressed(KEY_BACKSPACE) && !data_transfer_text.empty()) {
                data_transfer_text.pop_back();
            } else if (IsKeyPressed(KEY_ENTER) && !data_transfer_text.empty()) {
                try {
                    data_transfer_amount = std::stoi(data_transfer_text);
                    if (data_transfer_amount <= 0) {
                        data_transfer_amount = 0;
                        data_transfer_text = "";
                    }
                } catch (...) {
                    data_transfer_amount = 0;
                    data_transfer_text = "";
                }
            }
        }
        
        if (IsKeyPressed(KEY_ESCAPE)) {
            currentState = MENU;
            source_selected = false;
            target_selected = false;
            data_transfer_input_active = false;
        }
    }
    
    // Process Iteration Visualization input
    void ProcessVisualizeIteration() {
        // CRITICAL: Always update highlighted_path to show the most recent iteration
        // This ensures the final iteration is always visible, even after canContinue() returns false
        const auto& history = solver->getIterationHistory();
        if (!history.empty()) {
            // Get the path from the most recent iteration (always the last one in history)
            highlighted_path = history[history.size() - 1].path;
        }
        
        // Check if we can continue AFTER ensuring path is updated
        // NOTE: This check is only for UI button display, NOT for auto-transition
        bool canContinue = solver->canContinue();
        
        // Handle mouse clicks - ONLY way to change state
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector2 mousePos = GetMousePosition();
            Rectangle nextButton = {600, 520, 150, 50};
            Rectangle summaryButton = {600, 520, 150, 50};
            
            if (canContinue) {
                // Next Step button - run next iteration
                if (CheckCollisionPointRec(mousePos, nextButton)) {
                    // Run next iteration
                    if (solver->runStep()) {
                        // Update highlighted path to show the new iteration
                        const auto& newHistory = solver->getIterationHistory();
                        if (!newHistory.empty()) {
                            highlighted_path = newHistory[newHistory.size() - 1].path;
                        } else {
                            highlighted_path = solver->getPath();
                        }
                        // IMPORTANT: Stay in VISUALIZE_ITERATION state
                        // The final iteration will be displayed in the next frame
                        // User must explicitly click "Show Summary" to transition
                    }
                }
            } else {
                // Final iteration completed - Show Summary button is visible
                // CRITICAL: Only transition on explicit button click
                // The final iteration remains visible until user clicks this button
                if (CheckCollisionPointRec(mousePos, summaryButton)) {
                    currentState = SUMMARY;
                }
                // NO AUTO-TRANSITION - state stays as VISUALIZE_ITERATION until button click
            }
        }
        
        // ESC key goes back to node selection
        if (IsKeyPressed(KEY_ESCAPE)) {
            currentState = SELECT_NODES;
        }
        
        // CRITICAL: Never auto-transition to SUMMARY
        // The final iteration must remain visible until user explicitly clicks "Show Summary"
    }
    
    // Process Summary input
    void ProcessSummary() {
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Rectangle backButton = {600, 540, 150, 40};
            Vector2 mousePos = GetMousePosition();
            if (CheckCollisionPointRec(mousePos, backButton)) {
                currentState = VISUALIZE_ITERATION;
            }
        }
        
        if (IsKeyPressed(KEY_ESCAPE)) {
            currentState = VISUALIZE_ITERATION;
        }
    }
    // Setup demo network
    void setupDemoNetwork(Network& net) {
        // 5-city demo network
        net.setNodePosition(0, 200, 150);
        net.setNodePosition(1, 400, 150);
        net.setNodePosition(2, 600, 250);
        net.setNodePosition(3, 200, 350);
        net.setNodePosition(4, 400, 400);
        
        net.addUndirectedEdge(0, 1, 20);
        net.addUndirectedEdge(1, 2, 15);
        net.addUndirectedEdge(0, 3, 25);
        net.addUndirectedEdge(1, 4, 18);
        net.addUndirectedEdge(2, 4, 12);
        net.addUndirectedEdge(3, 4, 20);
    }
    
public:
    GUI(Network* net = nullptr, MaxCapacitySolver* sol = nullptr) 
        : network(net), solver(sol), currentState(MENU),
          source_node(-1), target_node(-1), source_selected(false), target_selected(false),
          data_transfer_input_active(false), data_transfer_amount(0) {
        if (network) {
            calculateNodePositions();
        }
        if (!solver && network) {
            solver = new MaxCapacitySolver(network);
        }
    }
    
    ~GUI() {
        // Cleanup handled by caller
    }
    
    void Run() {
        const int screenWidth = 800;
        const int screenHeight = 600;
        
        InitWindow(screenWidth, screenHeight, "Max Capacity Network Flow Visualizer");
        SetTargetFPS(60);
        
        while (!WindowShouldClose()) {
            BeginDrawing();
            ClearBackground(RAYWHITE);
            
            switch (currentState) {
                case MENU:
                    DrawMenu();
                    ProcessMenuInput();
                    break;
                case SELECT_NODES:
                    DrawSelectNodes();
                    ProcessSelectNodes();
                    break;
                case VISUALIZE_ITERATION:
                    DrawVisualizeIteration();
                    ProcessVisualizeIteration();
                    break;
                case SUMMARY:
                    DrawSummary();
                    ProcessSummary();
                    break;
            }
            
            EndDrawing();
        }
        
        CloseWindow();
    }
};
#endif
