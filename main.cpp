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
