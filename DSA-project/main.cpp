#include "raylib.h"
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <algorithm>
#include <limits>
#include <queue>
#include <cmath>

struct Edge {
    int to;
    int initial_capacity;
    int residual_capacity;
    int reverse_edge_index;
    
    Edge(int t, int cap) : to(t), initial_capacity(cap), residual_capacity(cap), reverse_edge_index(-1) {}
};

enum AppState {
    SETUP_CONFIG,
    INPUT_EDGES,
    SELECT_SOURCE_TARGET,
    SIMULATION_PAUSED,
    SIMULATION_STEP,
    RESULT_FINAL
};

class CheapestRoutingOptimizer {
private:
    // String-to-ID mapping
    std::map<std::string, int> cityToId;
    std::vector<std::string> idToCity;
    
    // Graph structure
    std::vector<std::vector<Edge>> graph;
    int numNodes;
    
    // Simulation parameters
    std::string sourceCity;
    std::string targetCity;
    int sourceId;
    int targetId;
    int totalData;
    int dataTransferred;
    int dataRemaining;
    int currentIteration;
    
    // Current path for visualization
    std::vector<int> currentPath;
    bool pathFound;
    
    // Iteration history
    std::vector<IterationData> iterationHistory;
    
    // Helper function to split comma-separated string
    std::vector<std::string> splitString(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, delimiter)) {
            // Trim whitespace
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (!token.empty()) {
                tokens.push_back(token);
            }
        }
        return tokens;
    }
    
    // Dijkstra's algorithm with cost function: 1.0 / (residual_capacity + 0.1)
    bool findCheapestPath(std::vector<int>& path, int& bottleneck, double& pathCost) {
        std::vector<double> dist(numNodes, std::numeric_limits<double>::max());
        std::vector<int> parent(numNodes, -1);
        std::vector<bool> visited(numNodes, false);
        
        std::priority_queue<std::pair<double, int>, 
                           std::vector<std::pair<double, int>>,
                           std::greater<std::pair<double, int>>> pq;
        
        dist[sourceId] = 0.0;
        pq.push({0.0, sourceId});
        
        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            
            if (visited[u]) continue;
            visited[u] = true;
            
            if (u == targetId) break;
            
            for (size_t i = 0; i < graph[u].size(); i++) {
                const Edge& e = graph[u][i];
                if (e.residual_capacity > 0) {
                    double cost = 1.0 / (e.residual_capacity + 0.1);
                    if (dist[u] + cost < dist[e.to]) {
                        dist[e.to] = dist[u] + cost;
                        parent[e.to] = u;
                        pq.push({dist[e.to], e.to});
                    }
                }
            }
        }
        
        if (parent[targetId] == -1) {
            return false; // No path found
        }
        
        // Reconstruct path and calculate total cost
        path.clear();
        int current = targetId;
        bottleneck = std::numeric_limits<int>::max();
        pathCost = dist[targetId]; // Total cost is the distance to target
        
        while (current != -1) {
            path.push_back(current);
            if (parent[current] != -1) {
                // Find edge capacity
                for (const Edge& e : graph[parent[current]]) {
                    if (e.to == current) {
                        bottleneck = std::min(bottleneck, e.residual_capacity);
                        break;
                    }
                }
            }
            current = parent[current];
        }
        
        std::reverse(path.begin(), path.end());
        return true;
    }
    
public:
    CheapestRoutingOptimizer() : numNodes(0), sourceId(-1), targetId(-1), 
                                  totalData(0), dataTransferred(0), dataRemaining(0),
                                  currentIteration(0), pathFound(false) {
        iterationHistory.clear();
    }
    
    // Initialize cities from comma-separated string
    bool initializeCities(const std::string& cityList) {
        std::vector<std::string> cities = splitString(cityList, ',');
        if (cities.size() < 2) {
            return false;
        }
        
        cityToId.clear();
        idToCity.clear();
        
        for (size_t i = 0; i < cities.size(); i++) {
            cityToId[cities[i]] = i;
            idToCity.push_back(cities[i]);
        }
        
        numNodes = cities.size();
        graph.clear();
        graph.resize(numNodes);
        
        return true;
    }
    
    // Add edge using city names
    bool addEdge(const std::string& fromCity, const std::string& toCity, int capacity) {
        if (cityToId.find(fromCity) == cityToId.end() || 
            cityToId.find(toCity) == cityToId.end()) {
            return false;
        }
        
        int from = cityToId[fromCity];
        int to = cityToId[toCity];
        
        if (from == to || capacity <= 0) {
            return false;
        }
        
        // Add forward edge
        int forwardIndex = graph[from].size();
        graph[from].push_back(Edge(to, capacity));
        
        // Add reverse edge (with 0 initial capacity)
        int reverseIndex = graph[to].size();
        graph[to].push_back(Edge(from, 0));
        
        // Link reverse edges
        graph[from][forwardIndex].reverse_edge_index = reverseIndex;
        graph[to][reverseIndex].reverse_edge_index = forwardIndex;
        
        return true;
    }
    
    // Set source, target, and total data
    bool setSourceTargetData(const std::string& src, const std::string& tgt, int data) {
        if (cityToId.find(src) == cityToId.end() || 
            cityToId.find(tgt) == cityToId.end()) {
            return false;
        }
        
        sourceCity = src;
        targetCity = tgt;
        sourceId = cityToId[src];
        targetId = cityToId[tgt];
        totalData = data;
        dataRemaining = data;
        dataTransferred = 0;
        currentIteration = 0;
        
        // Clear previous history when starting new simulation
        clearHistory();
        
        return true;
    }
    
    // Run one iteration of the flow algorithm
    bool runStep() {
        if (dataRemaining <= 0) {
            pathFound = false;
            currentPath.clear();
            return false; // All data transferred
        }
        
        std::vector<int> path;
        int bottleneck;
        double pathCost;
        
        if (!findCheapestPath(path, bottleneck, pathCost)) {
            pathFound = false;
            currentPath.clear();
            return false; // No path found
        }
        
        pathFound = true;
        currentPath = path;
        
        // Calculate flow: min(bottleneck, dataRemaining)
        int flow = std::min(bottleneck, dataRemaining);
        
        // Update residual capacities along the path
        for (size_t i = 0; i < path.size() - 1; i++) {
            int u = path[i];
            int v = path[i + 1];
            
            // Find and update forward edge
            for (Edge& e : graph[u]) {
                if (e.to == v) {
                    e.residual_capacity -= flow;
                    // Update reverse edge
                    if (e.reverse_edge_index >= 0) {
                        graph[v][e.reverse_edge_index].residual_capacity += flow;
                    }
                    break;
                }
            }
        }
        
        dataTransferred += flow;
        dataRemaining -= flow;
        currentIteration++;
        
        // Store iteration data
        IterationData iterData;
        iterData.iteration = currentIteration;
        iterData.path = path;
        iterData.flow = flow;
        iterData.dataTransferred = dataTransferred;
        iterData.dataRemaining = dataRemaining;
        iterData.cost = pathCost;
        iterationHistory.push_back(iterData);
        
        return true;
    }
    
    // Check if a path exists without running a step
    bool checkPathExists() {
        if (dataRemaining <= 0) return false;
        
        std::vector<int> testPath;
        int testBottleneck;
        double testCost;
        return findCheapestPath(testPath, testBottleneck, testCost);
    }
    
    // Getters
    const std::vector<int>& getCurrentPath() const { return currentPath; }
    bool hasPath() const { return pathFound; }
    int getDataTransferred() const { return dataTransferred; }
    int getDataRemaining() const { return dataRemaining; }
    int getTotalData() const { return totalData; }
    int getCurrentIteration() const { return currentIteration; }
    int getNumNodes() const { return numNodes; }
    const std::vector<std::string>& getCityNames() const { return idToCity; }
    const std::vector<std::vector<Edge>>& getGraph() const { return graph; }
    std::string getCityName(int id) const {
        if (id >= 0 && id < (int)idToCity.size()) {
            return idToCity[id];
        }
        return "";
    }
    
    // Check if city exists
    bool cityExists(const std::string& city) const {
        return cityToId.find(city) != cityToId.end();
    }
    
    // Get all city names
    std::vector<std::string> getAllCities() const {
        return idToCity;
    }
    
    // Get iteration history
    const std::vector<IterationData>& getIterationHistory() const {
        return iterationHistory;
    }
    
    // Clear iteration history (when starting new simulation)
    void clearHistory() {
        iterationHistory.clear();
        currentIteration = 0;
        dataTransferred = 0;
    }
};
class CheapestRoutingOptimizer {
private:
    // String-to-ID mapping
    std::map<std::string, int> cityToId;
    std::vector<std::string> idToCity;
    
    // Graph structure
    std::vector<std::vector<Edge>> graph;
    int numNodes;
    
    // Simulation parameters
    std::string sourceCity;
    std::string targetCity;
    int sourceId;
    int targetId;
    int totalData;
    int dataTransferred;
    int dataRemaining;
    int currentIteration;
    
    // Current path for visualization
    std::vector<int> currentPath;
    bool pathFound;
    
    // Iteration history
    std::vector<IterationData> iterationHistory;
    
    // Helper function to split comma-separated string
    std::vector<std::string> splitString(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, delimiter)) {
            // Trim whitespace
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (!token.empty()) {
                tokens.push_back(token);
            }
        }
        return tokens;
    }
    
    // Dijkstra's algorithm with cost function: 1.0 / (residual_capacity + 0.1)
    bool findCheapestPath(std::vector<int>& path, int& bottleneck, double& pathCost) {
        std::vector<double> dist(numNodes, std::numeric_limits<double>::max());
        std::vector<int> parent(numNodes, -1);
        std::vector<bool> visited(numNodes, false);
        
        std::priority_queue<std::pair<double, int>, 
                           std::vector<std::pair<double, int>>,
                           std::greater<std::pair<double, int>>> pq;
        
        dist[sourceId] = 0.0;
        pq.push({0.0, sourceId});
        
        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            
            if (visited[u]) continue;
            visited[u] = true;
            
            if (u == targetId) break;
            
            for (size_t i = 0; i < graph[u].size(); i++) {
                const Edge& e = graph[u][i];
                if (e.residual_capacity > 0) {
                    double cost = 1.0 / (e.residual_capacity + 0.1);
                    if (dist[u] + cost < dist[e.to]) {
                        dist[e.to] = dist[u] + cost;
                        parent[e.to] = u;
                        pq.push({dist[e.to], e.to});
                    }
                }
            }
        }
        
        if (parent[targetId] == -1) {
            return false; // No path found
        }
        
        // Reconstruct path and calculate total cost
        path.clear();
        int current = targetId;
        bottleneck = std::numeric_limits<int>::max();
        pathCost = dist[targetId]; // Total cost is the distance to target
        
        while (current != -1) {
            path.push_back(current);
            if (parent[current] != -1) {
                // Find edge capacity
                for (const Edge& e : graph[parent[current]]) {
                    if (e.to == current) {
                        bottleneck = std::min(bottleneck, e.residual_capacity);
                        break;
                    }
                }
            }
            current = parent[current];
        }
        
        std::reverse(path.begin(), path.end());
        return true;
    }
    
public:
    CheapestRoutingOptimizer() : numNodes(0), sourceId(-1), targetId(-1), 
                                  totalData(0), dataTransferred(0), dataRemaining(0),
                                  currentIteration(0), pathFound(false) {
        iterationHistory.clear();
    }
    
    // Initialize cities from comma-separated string
    bool initializeCities(const std::string& cityList) {
        std::vector<std::string> cities = splitString(cityList, ',');
        if (cities.size() < 2) {
            return false;
        }
        
        cityToId.clear();
        idToCity.clear();
        
        for (size_t i = 0; i < cities.size(); i++) {
            cityToId[cities[i]] = i;
            idToCity.push_back(cities[i]);
        }
        
        numNodes = cities.size();
        graph.clear();
        graph.resize(numNodes);
        
        return true;
    }
    
    // Add edge using city names
    bool addEdge(const std::string& fromCity, const std::string& toCity, int capacity) {
        if (cityToId.find(fromCity) == cityToId.end() || 
            cityToId.find(toCity) == cityToId.end()) {
            return false;
        }
        
        int from = cityToId[fromCity];
        int to = cityToId[toCity];
        
        if (from == to || capacity <= 0) {
            return false;
        }
        
        // Add forward edge
        int forwardIndex = graph[from].size();
        graph[from].push_back(Edge(to, capacity));
        
        // Add reverse edge (with 0 initial capacity)
        int reverseIndex = graph[to].size();
        graph[to].push_back(Edge(from, 0));
        
        // Link reverse edges
        graph[from][forwardIndex].reverse_edge_index = reverseIndex;
        graph[to][reverseIndex].reverse_edge_index = forwardIndex;
        
        return true;
    }
    
    // Set source, target, and total data
    bool setSourceTargetData(const std::string& src, const std::string& tgt, int data) {
        if (cityToId.find(src) == cityToId.end() || 
            cityToId.find(tgt) == cityToId.end()) {
            return false;
        }
        
        sourceCity = src;
        targetCity = tgt;
        sourceId = cityToId[src];
        targetId = cityToId[tgt];
        totalData = data;
        dataRemaining = data;
        dataTransferred = 0;
        currentIteration = 0;
        
        // Clear previous history when starting new simulation
        clearHistory();
        
        return true;
    }
    
    // Run one iteration of the flow algorithm
    bool runStep() {
        if (dataRemaining <= 0) {
            pathFound = false;
            currentPath.clear();
            return false; // All data transferred
        }
        
        std::vector<int> path;
        int bottleneck;
        double pathCost;
        
        if (!findCheapestPath(path, bottleneck, pathCost)) {
            pathFound = false;
            currentPath.clear();
            return false; // No path found
        }
        
        pathFound = true;
        currentPath = path;
        
        // Calculate flow: min(bottleneck, dataRemaining)
        int flow = std::min(bottleneck, dataRemaining);
        
        // Update residual capacities along the path
        for (size_t i = 0; i < path.size() - 1; i++) {
            int u = path[i];
            int v = path[i + 1];
            
            // Find and update forward edge
            for (Edge& e : graph[u]) {
                if (e.to == v) {
                    e.residual_capacity -= flow;
                    // Update reverse edge
                    if (e.reverse_edge_index >= 0) {
                        graph[v][e.reverse_edge_index].residual_capacity += flow;
                    }
                    break;
                }
            }
        }
        
        dataTransferred += flow;
        dataRemaining -= flow;
        currentIteration++;
        
        // Store iteration data
        IterationData iterData;
        iterData.iteration = currentIteration;
        iterData.path = path;
        iterData.flow = flow;
        iterData.dataTransferred = dataTransferred;
        iterData.dataRemaining = dataRemaining;
        iterData.cost = pathCost;
        iterationHistory.push_back(iterData);
        
        return true;
    }
    
    // Check if a path exists without running a step
    bool checkPathExists() {
        if (dataRemaining <= 0) return false;
        
        std::vector<int> testPath;
        int testBottleneck;
        double testCost;
        return findCheapestPath(testPath, testBottleneck, testCost);
    }
    
    // Getters
    const std::vector<int>& getCurrentPath() const { return currentPath; }
    bool hasPath() const { return pathFound; }
    int getDataTransferred() const { return dataTransferred; }
    int getDataRemaining() const { return dataRemaining; }
    int getTotalData() const { return totalData; }
    int getCurrentIteration() const { return currentIteration; }
    int getNumNodes() const { return numNodes; }
    const std::vector<std::string>& getCityNames() const { return idToCity; }
    const std::vector<std::vector<Edge>>& getGraph() const { return graph; }
    std::string getCityName(int id) const {
        if (id >= 0 && id < (int)idToCity.size()) {
            return idToCity[id];
        }
        return "";
    }
    
    // Check if city exists
    bool cityExists(const std::string& city) const {
        return cityToId.find(city) != cityToId.end();
    }
    
    // Get all city names
    std::vector<std::string> getAllCities() const {
        return idToCity;
    }
    
    // Get iteration history
    const std::vector<IterationData>& getIterationHistory() const {
        return iterationHistory;
    }
    
    // Clear iteration history (when starting new simulation)
    void clearHistory() {
        iterationHistory.clear();
        currentIteration = 0;
        dataTransferred = 0;
    }
};
// UI class 
class UI {
private:
    AppState currentState;
    CheapestRoutingOptimizer optimizer;
    
    // Input fields
    std::string cityInputText;
    std::string sourceInputText;
    std::string targetInputText;
    std::string dataInputText;
    std::string edgeFromText;
    std::string edgeToText;
    std::string edgeCapacityText;
    
    // Focus management
    int activeInputField; // 0=city, 1=source, 2=target, 3=data, 4=edgeFrom, 5=edgeTo, 6=edgeCap, -1=none
    
    // Dropdown states
    bool sourceDropdownOpen;
    bool targetDropdownOpen;
    
    // Status messages
    std::string statusMessage;
    Color statusColor;
    
    // Node positions for visualization
    std::vector<Vector2> nodePositions;
    
    // Auto-transition timer (reset when entering SIMULATION_STEP)
    float autoTransitionTimer;
    
    // Calculate node positions in circular layout
    void calculateNodePositions(int numNodes) {
        nodePositions.clear();
        if (numNodes == 0) return;
        
        float centerX = 400.0f;
        float centerY = 300.0f;
        float radius = 200.0f;
        
        for (int i = 0; i < numNodes; i++) {
            float angle = 2.0f * PI * i / numNodes - PI / 2.0f;
            nodePositions.push_back({
                centerX + radius * cosf(angle),
                centerY + radius * sinf(angle)
            });
        }
    }
    
    // Draw the graph
    void drawGraph(bool highlightPath = false) {
        if (optimizer.getNumNodes() == 0) return;
        
        const auto& graph = optimizer.getGraph();
        const auto& cityNames = optimizer.getCityNames();
        
        // Get the path to highlight (from iteration history if available, otherwise from current path)
        std::vector<int> pathToHighlight;
        if (highlightPath) {
            const auto& history = optimizer.getIterationHistory();
            int currentIter = optimizer.getCurrentIteration();
            if (!history.empty() && currentIter > 0 && currentIter <= (int)history.size()) {
                // Use path from the most recent completed iteration
                pathToHighlight = history[currentIter - 1].path;
            } else if (optimizer.hasPath()) {
                // Fallback to current path
                pathToHighlight = optimizer.getCurrentPath();
            }
        }
        
        // Draw edges
        for (int i = 0; i < optimizer.getNumNodes(); i++) {
            for (const Edge& e : graph[i]) {
                if (e.initial_capacity > 0) { // Only draw forward edges
                    Vector2 fromPos = nodePositions[i];
                    Vector2 toPos = nodePositions[e.to];
                    
                    // Check if this edge is on the path to highlight
                    bool isPathEdge = false;
                    if (highlightPath && !pathToHighlight.empty()) {
                        for (size_t j = 0; j < pathToHighlight.size() - 1; j++) {
                            if ((pathToHighlight[j] == i && pathToHighlight[j + 1] == e.to)) {
                                isPathEdge = true;
                                break;
                            }
                        }
                    }
                    
                    // Draw edge
                    Color edgeColor = isPathEdge ? GOLD : DARKGRAY;
                    float thickness = isPathEdge ? 4.0f : 2.0f;
                    DrawLineEx(fromPos, toPos, thickness, edgeColor);
                    
                    // Draw capacity label
                    std::string capStr = std::to_string(e.residual_capacity) + "/" + std::to_string(e.initial_capacity);
                    Vector2 midPoint = {(fromPos.x + toPos.x) / 2.0f, (fromPos.y + toPos.y) / 2.0f};
                    Vector2 textSize = MeasureTextEx(GetFontDefault(), capStr.c_str(), 10, 1.0f);
                    DrawRectangle(midPoint.x - textSize.x/2 - 2, midPoint.y - textSize.y/2 - 2,
                                 textSize.x + 4, textSize.y + 4, WHITE);
                    DrawText(capStr.c_str(), midPoint.x - textSize.x/2, midPoint.y - textSize.y/2, 10, BLACK);
                }
            }
        }
        
        // Draw nodes
        for (int i = 0; i < optimizer.getNumNodes(); i++) {
            Vector2 pos = nodePositions[i];
            std::string cityName = cityNames[i];
            
            // Check if node is on path
            bool isPathNode = false;
            if (highlightPath && !pathToHighlight.empty()) {
                isPathNode = std::find(pathToHighlight.begin(), pathToHighlight.end(), i) != pathToHighlight.end();
            }
            
            Color nodeColor = isPathNode ? GOLD : LIGHTGRAY;
            DrawCircle(pos.x, pos.y, 25, nodeColor);
            DrawCircleLines(pos.x, pos.y, 25, BLACK);
            
            // Draw city name
            Vector2 textSize = MeasureTextEx(GetFontDefault(), cityName.c_str(), 14, 1.0f);
            DrawText(cityName.c_str(), pos.x - textSize.x/2, pos.y - textSize.y/2, 14, BLACK);
        }
    }
    
    // Draw status panel
    void drawStatusPanel() {
        int panelY = 550;
        DrawRectangle(10, panelY, 780, 40, Fade(LIGHTGRAY, 0.8f));
        DrawRectangleLines(10, panelY, 780, 40, BLACK);
        
        std::string status = "Data Requested: " + std::to_string(optimizer.getTotalData()) +
                           " | Data Transferred: " + std::to_string(optimizer.getDataTransferred()) +
                           " | Data Remaining: " + std::to_string(optimizer.getDataRemaining()) +
                           " | Iteration: " + std::to_string(optimizer.getCurrentIteration());
        DrawText(status.c_str(), 20, panelY + 10, 16, BLACK);
    }
    
    void drawSetupConfig() {
        DrawText("Cheapest Data Routing Optimizer", 200, 50, 28, BLACK);
        DrawText("Setup Configuration - Step 1: Map Cities", 250, 100, 24, DARKGRAY);
        
        // City input
        DrawText("Enter city names (comma-separated):", 50, 150, 18, DARKGRAY);
        Rectangle cityBox = {50, 180, 700, 30};
        Color cityBoxColor = (activeInputField == 0) ? YELLOW : LIGHTGRAY;
        DrawRectangleRec(cityBox, cityBoxColor);
        DrawRectangleLinesEx(cityBox, 2, DARKGRAY);
        DrawText(cityInputText.c_str(), 60, 187, 18, BLACK);
        
        // Map Cities button
        Rectangle mapButton = {50, 230, 150, 40};
        Color mapButtonColor = CheckCollisionPointRec(GetMousePosition(), mapButton) ? GRAY : DARKGRAY;
        DrawRectangleRec(mapButton, mapButtonColor);
        DrawRectangleLinesEx(mapButton, 2, BLACK);
        DrawText("Map Cities", 70, 240, 18, WHITE);
        
        // Load Demo button
        Rectangle demoButton = {500, 220, 200, 50};
        Color demoButtonColor = CheckCollisionPointRec(GetMousePosition(), demoButton) ? GRAY : GREEN;
        DrawRectangleRec(demoButton, demoButtonColor);
        DrawRectangleLinesEx(demoButton, 2, BLACK);
        DrawText("Load 5-City Demo", 520, 240, 18, WHITE);
        
        // Status message
        DrawText(statusMessage.c_str(), 50, 290, 18, statusColor);
        
        // Continue to Input Edges button (only if cities are mapped)
        if (optimizer.getNumNodes() > 0) {
            Rectangle continueButton = {600, 280, 150, 40};
            Color continueButtonColor = CheckCollisionPointRec(GetMousePosition(), continueButton) ? GRAY : BLUE;
            DrawRectangleRec(continueButton, continueButtonColor);
            DrawRectangleLinesEx(continueButton, 2, BLACK);
            DrawText("Input Edges", 620, 290, 18, WHITE);
        }
    }
    
    void drawSelectSourceTarget() {
        DrawText("Select Source, Target, and Data", 200, 50, 28, BLACK);
        DrawText("Step 3: Configure Simulation Parameters", 250, 100, 24, DARKGRAY);
        
        // Source dropdown
        DrawText("Source City:", 50, 150, 18, DARKGRAY);
        Rectangle sourceBox = {200, 145, 200, 30};
        Color sourceBoxColor = sourceDropdownOpen ? YELLOW : LIGHTGRAY;
        DrawRectangleRec(sourceBox, sourceBoxColor);
        DrawRectangleLinesEx(sourceBox, 2, DARKGRAY);
        DrawText((sourceInputText.empty() ? "Select..." : sourceInputText.c_str()), 210, 152, 18, BLACK);
        DrawText("▼", 380, 152, 16, DARKGRAY);
        
        if (sourceDropdownOpen && optimizer.getNumNodes() > 0) {
            const auto& cities = optimizer.getAllCities();
            Rectangle dropdownRect = {200.0f, 175.0f, 200.0f, static_cast<float>(cities.size()) * 25.0f + 10.0f};
            DrawRectangleRec(dropdownRect, WHITE);
            DrawRectangleLinesEx(dropdownRect, 2, DARKGRAY);
            for (size_t i = 0; i < cities.size(); i++) {
                Rectangle itemRect = {202.0f, 177.0f + static_cast<float>(i) * 25.0f, 196.0f, 23.0f};
                Color itemColor = CheckCollisionPointRec(GetMousePosition(), itemRect) ? LIGHTGRAY : WHITE;
                DrawRectangleRec(itemRect, itemColor);
                DrawText(cities[i].c_str(), 210, static_cast<int>(180.0f + static_cast<float>(i) * 25.0f), 16, BLACK);
            }
        }
        
        // Target dropdown
        DrawText("Target City:", 50, 200, 18, DARKGRAY);
        Rectangle targetBox = {200, 195, 200, 30};
        Color targetBoxColor = targetDropdownOpen ? YELLOW : LIGHTGRAY;
        DrawRectangleRec(targetBox, targetBoxColor);
        DrawRectangleLinesEx(targetBox, 2, DARKGRAY);
        DrawText((targetInputText.empty() ? "Select..." : targetInputText.c_str()), 210, 202, 18, BLACK);
        DrawText("▼", 380, 202, 16, DARKGRAY);
        
        if (targetDropdownOpen && optimizer.getNumNodes() > 0) {
            const auto& cities = optimizer.getAllCities();
            Rectangle dropdownRect = {200.0f, 225.0f, 200.0f, static_cast<float>(cities.size()) * 25.0f + 10.0f};
            DrawRectangleRec(dropdownRect, WHITE);
            DrawRectangleLinesEx(dropdownRect, 2, DARKGRAY);
            for (size_t i = 0; i < cities.size(); i++) {
                Rectangle itemRect = {202.0f, 227.0f + static_cast<float>(i) * 25.0f, 196.0f, 23.0f};
                Color itemColor = CheckCollisionPointRec(GetMousePosition(), itemRect) ? LIGHTGRAY : WHITE;
                DrawRectangleRec(itemRect, itemColor);
                DrawText(cities[i].c_str(), 210, static_cast<int>(230.0f + static_cast<float>(i) * 25.0f), 16, BLACK);
            }
        }
        
        // Data input (typing format)
        DrawText("Total Data to Transfer:", 50, 250, 18, DARKGRAY);
        Rectangle dataBox = {250, 245, 150, 30};
        Color dataBoxColor = (activeInputField == 3) ? YELLOW : LIGHTGRAY;
        DrawRectangleRec(dataBox, dataBoxColor);
        DrawRectangleLinesEx(dataBox, 2, DARKGRAY);
        DrawText((dataInputText.empty() ? "Enter amount..." : dataInputText.c_str()), 260, 252, 18, BLACK);
        
        // Status message
        DrawText(statusMessage.c_str(), 50, 300, 18, statusColor);
        
        // Continue to Simulation button (only if all set)
        if (!sourceInputText.empty() && !targetInputText.empty() && !dataInputText.empty()) {
            Rectangle continueButton = {600, 400, 150, 40};
            Color continueButtonColor = CheckCollisionPointRec(GetMousePosition(), continueButton) ? GRAY : GREEN;
            DrawRectangleRec(continueButton, continueButtonColor);
            DrawRectangleLinesEx(continueButton, 2, BLACK);
            DrawText("Start Simulation", 610, 410, 18, WHITE);
        }
    }
    
    void drawInputEdges() {
        DrawText("Input Edges", 300, 30, 24, DARKGRAY);
        DrawText("Step 2: Add Network Connections", 250, 60, 20, DARKGRAY);
        
        DrawText("From City:", 50, 100, 18, DARKGRAY);
        Rectangle fromBox = {200, 95, 200, 30};
        Color fromBoxColor = (activeInputField == 4) ? YELLOW : LIGHTGRAY;
        DrawRectangleRec(fromBox, fromBoxColor);
        DrawRectangleLinesEx(fromBox, 2, DARKGRAY);
        DrawText(edgeFromText.c_str(), 210, 102, 18, BLACK);
        
        DrawText("To City:", 50, 140, 18, DARKGRAY);
        Rectangle toBox = {200, 135, 200, 30};
        Color toBoxColor = (activeInputField == 5) ? YELLOW : LIGHTGRAY;
        DrawRectangleRec(toBox, toBoxColor);
        DrawRectangleLinesEx(toBox, 2, DARKGRAY);
        DrawText(edgeToText.c_str(), 210, 142, 18, BLACK);
        
        DrawText("Capacity:", 50, 180, 18, DARKGRAY);
        Rectangle capBox = {200, 175, 150, 30};
        Color capBoxColor = (activeInputField == 6) ? YELLOW : LIGHTGRAY;
        DrawRectangleRec(capBox, capBoxColor);
        DrawRectangleLinesEx(capBox, 2, DARKGRAY);
        DrawText(edgeCapacityText.c_str(), 210, 182, 18, BLACK);
        
        // Add Edge button
        Rectangle addButton = {50, 230, 150, 40};
        Color addButtonColor = CheckCollisionPointRec(GetMousePosition(), addButton) ? GRAY : DARKGRAY;
        DrawRectangleRec(addButton, addButtonColor);
        DrawRectangleLinesEx(addButton, 2, BLACK);
        DrawText("Add Edge", 70, 240, 18, WHITE);
        
        // Continue to Select Source/Target button
        Rectangle nextButton = {500, 400, 200, 50};
        Color nextButtonColor = CheckCollisionPointRec(GetMousePosition(), nextButton) ? GRAY : GREEN;
        DrawRectangleRec(nextButton, nextButtonColor);
        DrawRectangleLinesEx(nextButton, 2, BLACK);
        DrawText("Next: Select Source/Target", 510, 415, 16, WHITE);
        
        DrawText(statusMessage.c_str(), 50, 280, 18, statusColor);
    }
    
    void drawSimulationPaused() {
        DrawText("Simulation Ready", 300, 30, 24, BLACK);
        drawGraph(false);
        drawStatusPanel();
        
        Rectangle nextButton = {600, 500, 150, 40};
        Color nextButtonColor = CheckCollisionPointRec(GetMousePosition(), nextButton) ? GRAY : GREEN;
        DrawRectangleRec(nextButton, nextButtonColor);
        DrawRectangleLinesEx(nextButton, 2, BLACK);
        DrawText("Next Step", 620, 510, 18, WHITE);
    }
    
    void drawSimulationStep() {
        // Always show the graph with current path highlighted
        drawGraph(true); // Highlight path
        drawStatusPanel();
        
        // Show current path from the last completed iteration
        // CRITICAL: Always show the most recent iteration from history
        const auto& history = optimizer.getIterationHistory();
        std::string title = "Simulation Step";
        
        if (!history.empty()) {
            // Always display the most recent iteration (last one in history)
            int iterIndex = history.size() - 1;
            const auto& iter = history[iterIndex];
            
            // Update title to show the actual iteration number
            title = "Simulation Step - Iteration " + std::to_string(iter.iteration);
            
            std::string pathStr = "Path: ";
            for (size_t i = 0; i < iter.path.size(); i++) {
                pathStr += optimizer.getCityName(iter.path[i]);
                if (i < iter.path.size() - 1) pathStr += " -> ";
            }
            DrawText(pathStr.c_str(), 50, 480, 16, DARKGRAY);
            
            // Show flow information for this iteration
            std::string flowInfo = "Flow: " + std::to_string(iter.flow) + 
                                  " | Data Transferred: " + std::to_string(iter.dataTransferred) +
                                  " | Remaining: " + std::to_string(iter.dataRemaining);
            DrawText(flowInfo.c_str(), 50, 500, 14, DARKGRAY);
        } else if (optimizer.hasPath()) {
            // Fallback: use current path if history is not available
            const auto& path = optimizer.getCurrentPath();
            std::string pathStr = "Path: ";
            for (size_t i = 0; i < path.size(); i++) {
                pathStr += optimizer.getCityName(path[i]);
                if (i < path.size() - 1) pathStr += " -> ";
            }
            DrawText(pathStr.c_str(), 50, 480, 16, DARKGRAY);
        }
        
        // Draw title after determining iteration number
        DrawText(title.c_str(), 250, 30, 24, BLACK);
        
        // Check if we can continue to next step
        bool canContinue = (optimizer.getDataRemaining() > 0 && optimizer.checkPathExists());
        
        if (canContinue) {
            Rectangle nextButton = {600, 500, 150, 40};
            Color nextButtonColor = CheckCollisionPointRec(GetMousePosition(), nextButton) ? GRAY : GREEN;
            DrawRectangleRec(nextButton, nextButtonColor);
            DrawRectangleLinesEx(nextButton, 2, BLACK);
            DrawText("Next Step", 620, 510, 18, WHITE);
        } else {
            // Final iteration completed - show completion message and View Results button
            // IMPORTANT: The final iteration is displayed above and remains visible
            // User must explicitly click "View Results" to proceed
            DrawText("Simulation Complete!", 50, 520, 20, GREEN);
            if (optimizer.getDataRemaining() > 0) {
                DrawText("No more paths available.", 50, 545, 16, DARKGRAY);
            } else {
                DrawText("All data transferred successfully!", 50, 545, 16, DARKGRAY);
            }
            
            // View Results button - user must click this to see summary
            Rectangle viewResultsButton = {600, 500, 150, 40};
            Color viewResultsButtonColor = CheckCollisionPointRec(GetMousePosition(), viewResultsButton) ? GRAY : BLUE;
            DrawRectangleRec(viewResultsButton, viewResultsButtonColor);
            DrawRectangleLinesEx(viewResultsButton, 2, BLACK);
            DrawText("View Results", 610, 510, 18, WHITE);
        }
    }
    
    void drawResultFinal() {
        DrawText("Simulation Complete", 250, 10, 28, BLACK);
        
        // Summary section
        int y = 50;
        DrawText("=== Final Summary ===", 50, y, 22, DARKBLUE);
        y += 30;
        DrawText(("Total Data Requested: " + std::to_string(optimizer.getTotalData())).c_str(), 50, y, 18, BLACK);
        y += 25;
        DrawText(("Total Data Transferred: " + std::to_string(optimizer.getDataTransferred())).c_str(), 50, y, 18, BLACK);
        y += 25;
        DrawText(("Data Remaining: " + std::to_string(optimizer.getDataRemaining())).c_str(), 50, y, 18, BLACK);
        y += 25;
        DrawText(("Total Iterations: " + std::to_string(optimizer.getCurrentIteration())).c_str(), 50, y, 18, BLACK);
        y += 25;
        
        const auto& history = optimizer.getIterationHistory();
        
        float efficiency = optimizer.getTotalData() > 0 ? 
            (100.0f * optimizer.getDataTransferred() / optimizer.getTotalData()) : 0.0f;
        DrawText(("Transfer Efficiency: " + std::to_string((int)efficiency) + "%").c_str(), 50, y, 18, BLACK);
        y += 40;
        
        // Iteration history
        if (!history.empty()) {
            DrawText("=== Iteration Details ===", 50, y, 22, DARKBLUE);
            y += 30;
            
            // Scrollable area for iterations
            int startY = y;
            int maxHeight = 400;
            int scrollOffset = 0;
            
            for (size_t i = 0; i < history.size(); i++) {
                const auto& iter = history[i];
                int currentY = startY + (int)i * 60 - scrollOffset;
                
                if (currentY < startY - 20 || currentY > startY + maxHeight) continue;
                
                // Draw iteration box
                DrawRectangle(45, currentY - 5, 710, 55, Fade(LIGHTGRAY, 0.3f));
                DrawRectangleLines(45, currentY - 5, 710, 55, DARKGRAY);
                
                // Iteration number
                DrawText(("Iteration " + std::to_string(iter.iteration) + ":").c_str(), 50, currentY, 16, DARKBLUE);
                
                // Path
                std::string pathStr = "Path: ";
                for (size_t j = 0; j < iter.path.size(); j++) {
                    pathStr += optimizer.getCityName(iter.path[j]);
                    if (j < iter.path.size() - 1) pathStr += " -> ";
                }
                DrawText(pathStr.c_str(), 50, currentY + 18, 14, BLACK);
                
                // Flow and status
                std::string flowStr = "Flow: " + std::to_string(iter.flow) + 
                                     " | Transferred: " + std::to_string(iter.dataTransferred) +
                                     " | Remaining: " + std::to_string(iter.dataRemaining);
                DrawText(flowStr.c_str(), 50, currentY + 35, 12, DARKGRAY);
            }
        }
    }
    
    void processSetupConfig() {
        Vector2 mousePos = GetMousePosition();
        
        // Handle mouse clicks for city input
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Rectangle cityBox = {50, 180, 700, 30};
            if (CheckCollisionPointRec(mousePos, cityBox)) {
                activeInputField = 0;
            }
        }
        
        // Handle text input for city list
        int key = GetCharPressed();
        if (key > 0 && key < 128 && activeInputField == 0) {
            if ((key >= 'a' && key <= 'z') || (key >= 'A' && key <= 'Z') || 
                (key >= '0' && key <= '9') || key == ' ' || key == ',') {
                cityInputText += (char)key;
            }
        }
        
        if (IsKeyPressed(KEY_BACKSPACE) && activeInputField == 0) {
            if (!cityInputText.empty()) cityInputText.pop_back();
        }
        
        // Map Cities button
        Rectangle mapButton = {50, 230, 150, 40};
        if (CheckCollisionPointRec(GetMousePosition(), mapButton) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (optimizer.initializeCities(cityInputText)) {
                statusMessage = "Cities mapped successfully! Click 'Input Edges' to continue.";
                statusColor = GREEN;
                calculateNodePositions(optimizer.getNumNodes());
            } else {
                statusMessage = "Error: Invalid city list. Use comma-separated names.";
                statusColor = RED;
            }
        }
        
        // Load Demo button
        Rectangle demoButton = {500, 220, 200, 50};
        if (CheckCollisionPointRec(GetMousePosition(), demoButton) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            loadDemo();
        }
        
        // Continue to Input Edges button
        if (optimizer.getNumNodes() > 0) {
            Rectangle continueButton = {600, 280, 150, 40};
            if (CheckCollisionPointRec(GetMousePosition(), continueButton) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                currentState = INPUT_EDGES;
                statusMessage = "";
            }
        }
    }
    
    void processSelectSourceTarget() {
        Vector2 mousePos = GetMousePosition();
        
        // Handle mouse clicks for focus
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Rectangle sourceBox = {200, 145, 200, 30};
            Rectangle targetBox = {200, 195, 200, 30};
            Rectangle dataBox = {250, 245, 150, 30};
            
            // Handle source dropdown
            if (CheckCollisionPointRec(mousePos, sourceBox)) {
                if (optimizer.getNumNodes() > 0) {
                    sourceDropdownOpen = !sourceDropdownOpen;
                    targetDropdownOpen = false;
                    activeInputField = 1;
                }
            }
            // Handle source dropdown items
            else if (sourceDropdownOpen && optimizer.getNumNodes() > 0) {
                const auto& cities = optimizer.getAllCities();
                for (size_t i = 0; i < cities.size(); i++) {
                    Rectangle itemRect = {202.0f, 177.0f + static_cast<float>(i) * 25.0f, 196.0f, 23.0f};
                    if (CheckCollisionPointRec(mousePos, itemRect)) {
                        sourceInputText = cities[i];
                        sourceDropdownOpen = false;
                        break;
                    }
                }
            }
            // Handle target dropdown
            else if (CheckCollisionPointRec(mousePos, targetBox)) {
                if (optimizer.getNumNodes() > 0) {
                    targetDropdownOpen = !targetDropdownOpen;
                    sourceDropdownOpen = false;
                    activeInputField = 2;
                }
            }
            // Handle target dropdown items
            else if (targetDropdownOpen && optimizer.getNumNodes() > 0) {
                const auto& cities = optimizer.getAllCities();
                for (size_t i = 0; i < cities.size(); i++) {
                    Rectangle itemRect = {202.0f, 227.0f + static_cast<float>(i) * 25.0f, 196.0f, 23.0f};
                    if (CheckCollisionPointRec(mousePos, itemRect)) {
                        targetInputText = cities[i];
                        targetDropdownOpen = false;
                        break;
                    }
                }
            }
            // Handle data input box (typing format)
            else if (CheckCollisionPointRec(mousePos, dataBox)) {
                activeInputField = 3;
                sourceDropdownOpen = false;
                targetDropdownOpen = false;
            }
            // Close dropdowns if clicking outside
            else {
                sourceDropdownOpen = false;
                targetDropdownOpen = false;
                if (!CheckCollisionPointRec(mousePos, dataBox)) {
                    activeInputField = -1;
                }
            }
            
            // Start Simulation button
            if (!sourceInputText.empty() && !targetInputText.empty() && !dataInputText.empty()) {
                Rectangle continueButton = {600, 400, 150, 40};
                if (CheckCollisionPointRec(mousePos, continueButton)) {
                    try {
                        int data = std::stoi(dataInputText);
                        if (optimizer.setSourceTargetData(sourceInputText, targetInputText, data)) {
                            currentState = SIMULATION_PAUSED;
                            statusMessage = "";
                        } else {
                            statusMessage = "Error: Invalid source or target city.";
                            statusColor = RED;
                        }
                    } catch (...) {
                        statusMessage = "Error: Invalid data amount.";
                        statusColor = RED;
                    }
                }
            }
        }
        
        // Handle text input for data field
        int key = GetCharPressed();
        if (key > 0 && key < 128 && activeInputField == 3) {
            // Only numbers for data amount
            if (key >= '0' && key <= '9') {
                dataInputText += (char)key;
            }
        }
        
        if (IsKeyPressed(KEY_BACKSPACE) && activeInputField == 3) {
            if (!dataInputText.empty()) dataInputText.pop_back();
        }
    }
    
    void processInputEdges() {
        // Handle mouse clicks for focus
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector2 mousePos = GetMousePosition();
            Rectangle fromBox = {200, 95, 200, 30};
            Rectangle toBox = {200, 135, 200, 30};
            Rectangle capBox = {200, 175, 150, 30};
            
            if (CheckCollisionPointRec(mousePos, fromBox)) activeInputField = 4;
            else if (CheckCollisionPointRec(mousePos, toBox)) activeInputField = 5;
            else if (CheckCollisionPointRec(mousePos, capBox)) activeInputField = 6;
        }
        
        // Handle text input
        int key = GetCharPressed();
        if (key > 0 && key < 128) {
            std::string* activeField = nullptr;
            if (activeInputField == 4) activeField = &edgeFromText;
            else if (activeInputField == 5) activeField = &edgeToText;
            else if (activeInputField == 6) activeField = &edgeCapacityText;
            
            if (activeField) {
                if (activeInputField == 6) {
                    // Only numbers for capacity
                    if (key >= '0' && key <= '9') {
                        *activeField += (char)key;
                    }
                } else {
                    // Allow letters, numbers, and spaces for city names
                    if ((key >= 'a' && key <= 'z') || (key >= 'A' && key <= 'Z') || 
                        (key >= '0' && key <= '9') || key == ' ') {
                        *activeField += (char)key;
                    }
                }
            }
        }
        
        if (IsKeyPressed(KEY_BACKSPACE)) {
            if (activeInputField == 4 && !edgeFromText.empty()) edgeFromText.pop_back();
            else if (activeInputField == 5 && !edgeToText.empty()) edgeToText.pop_back();
            else if (activeInputField == 6 && !edgeCapacityText.empty()) edgeCapacityText.pop_back();
        }
        
        // Tab to cycle through fields
        if (IsKeyPressed(KEY_TAB)) {
            activeInputField = 4 + ((activeInputField - 4 + 1) % 3);
        }
        
        // Add Edge button
        Rectangle addButton = {50, 230, 150, 40};
        if (CheckCollisionPointRec(GetMousePosition(), addButton) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (!edgeFromText.empty() && !edgeToText.empty() && !edgeCapacityText.empty()) {
                try {
                    int capacity = std::stoi(edgeCapacityText);
                    if (optimizer.addEdge(edgeFromText, edgeToText, capacity)) {
                        statusMessage = "Edge added successfully!";
                        statusColor = GREEN;
                        edgeFromText = "";
                        edgeToText = "";
                        edgeCapacityText = "";
                    } else {
                        statusMessage = "Error: Invalid edge. Check city names and capacity.";
                        statusColor = RED;
                    }
                } catch (...) {
                    statusMessage = "Error: Invalid capacity value.";
                    statusColor = RED;
                }
            }
        }
        
        // Next: Select Source/Target button
        Rectangle nextButton = {500, 400, 200, 50};
        if (CheckCollisionPointRec(GetMousePosition(), nextButton) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            currentState = SELECT_SOURCE_TARGET;
            sourceInputText = "";
            targetInputText = "";
            dataInputText = "";
            statusMessage = "";
        }
    }
    
    void processSimulationPaused() {
        Rectangle nextButton = {600, 500, 150, 40};
        if (CheckCollisionPointRec(GetMousePosition(), nextButton) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            // Reset timer when starting simulation
            autoTransitionTimer = 0.0f;
            
            if (optimizer.runStep()) {
                currentState = SIMULATION_STEP;
            } else {
                // No path found even for first step, go directly to results
                currentState = RESULT_FINAL;
            }
        }
    }
    
    void processSimulationStep() {
        // Check if we can continue to next step
        bool buttonVisible = (optimizer.getDataRemaining() > 0 && optimizer.checkPathExists());
        
        // CRITICAL: Do NOT auto-transition to results
        // The final iteration must remain visible until user explicitly proceeds
        // Only transition on explicit user action (button click)
        
        // Handle Next Step button click (only if path exists)
        Rectangle nextButton = {600, 500, 150, 40};
        if (CheckCollisionPointRec(GetMousePosition(), nextButton) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            // Reset timer when starting a new step
            autoTransitionTimer = 0.0f;
            
            // Run the next step
            if (optimizer.runStep()) {
                // Successfully ran step, stay in SIMULATION_STEP to show the new path
                // The path will be displayed in drawSimulationStep()
                // IMPORTANT: Even if this is the final iteration, stay in SIMULATION_STEP
                // to display it. The next frame will check buttonVisible and show
                // completion message, but iteration will still be visible.
            } else {
                // Step failed (no path found or no data remaining)
                // Stay in SIMULATION_STEP to show the final iteration that was just completed
                // The drawSimulationStep() will show completion message
                // User can see the final iteration before any transition
            }
        }
        
        // Check if we should show "View Results" button instead of "Next Step"
        // But DO NOT auto-transition - let user see final iteration
        if (!buttonVisible && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            // Check if user clicked on a "View Results" area (if we add such a button)
            // For now, we'll add a button in drawSimulationStep() when !buttonVisible
            Rectangle viewResultsButton = {600, 500, 150, 40};
            if (CheckCollisionPointRec(GetMousePosition(), viewResultsButton)) {
                currentState = RESULT_FINAL;
                autoTransitionTimer = 0.0f;
            }
        }
        
        // Reset timer when button is visible (new step available)
        autoTransitionTimer = 0.0f;
    }
    
    void loadDemo() {
        // Initialize 5 cities (reduced from 6 to avoid overlapping capacity values)
        optimizer.initializeCities("Delhi, Mumbai, Hyderabad, Jaipur, Bengaluru");
        calculateNodePositions(5);
        
        // Add edges (directed, capacities 100-500)
        optimizer.addEdge("Delhi", "Mumbai", 300);
        optimizer.addEdge("Delhi", "Hyderabad", 250);
        optimizer.addEdge("Delhi", "Jaipur", 500);
        optimizer.addEdge("Mumbai", "Hyderabad", 200);
        optimizer.addEdge("Mumbai", "Jaipur", 400);
        optimizer.addEdge("Mumbai", "Bengaluru", 450);
        optimizer.addEdge("Hyderabad", "Bengaluru", 350);
        optimizer.addEdge("Jaipur", "Delhi", 180);
        
        // Update input text fields for display (but don't set source/target/data)
        cityInputText = "Delhi, Mumbai, Hyderabad, Jaipur, Bengaluru";
        // Leave source, target, and data empty so user can select them
        
        statusMessage = "Demo graph loaded with default edges! Please select Source, Target, and Data amount.";
        statusColor = GREEN;
        // Go to SELECT_SOURCE_TARGET so user can select source, target, and data
        sourceInputText = "";
        targetInputText = "";
        dataInputText = "";
        currentState = SELECT_SOURCE_TARGET;
    }
    
public:
    UI() : currentState(SETUP_CONFIG), activeInputField(0), 
          sourceDropdownOpen(false), targetDropdownOpen(false),
          statusColor(BLACK), autoTransitionTimer(0.0f) {
        calculateNodePositions(0);
    }
    
    void update() {
        switch (currentState) {
            case SETUP_CONFIG:
                processSetupConfig();
                break;
            case INPUT_EDGES:
                processInputEdges();
                break;
            case SELECT_SOURCE_TARGET:
                processSelectSourceTarget();
                break;
            case SIMULATION_PAUSED:
                processSimulationPaused();
                break;
            case SIMULATION_STEP:
                processSimulationStep();
                break;
            case RESULT_FINAL:
                // No processing needed
                break;
        }
    }
    
    void render() {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        switch (currentState) {
            case SETUP_CONFIG:
                drawSetupConfig();
                break;
            case INPUT_EDGES:
                drawInputEdges();
                break;
            case SELECT_SOURCE_TARGET:
                drawSelectSourceTarget();
                break;
            case SIMULATION_PAUSED:
                drawSimulationPaused();
                break;
            case SIMULATION_STEP:
                drawSimulationStep();
                break;
            case RESULT_FINAL:
                drawResultFinal();
                break;
        }
        
        EndDrawing();
    }
    
    void run() {
        const int screenWidth = 800;
        const int screenHeight = 600;
        
        InitWindow(screenWidth, screenHeight, "Cheapest Data Routing Optimizer");
        SetTargetFPS(60);
        
        while (!WindowShouldClose()) {
            update();
            render();
        }
        
        CloseWindow();
    }
};
int main() {
    UI ui;
    ui.run();
    return 0;
}
