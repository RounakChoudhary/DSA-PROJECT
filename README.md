# 🌐 Network Optimization: Data Congestion & Cheapest Path Solver (DSA-PROJECT)

## 🌟 Project Overview

This repository contains a C++ application for modeling and optimizing a network using graph algorithms. The project minimizes data congestion and finds the most efficient path for transferring data between nodes while visualizing the network through a Raylib-based GUI.

Key capabilities:

* Create and edit directed network graphs.
* Run max-flow / min-cut optimization.
* Compute the cheapest path between source and target nodes.
* Visualize the network and simulation steps.
* Demo modes, including a deterministic 50-city example.

---

## ✨ Features

* Max-flow and congestion-aware routing.
* Cheapest path selection over the network.
* Raylib GUI for input, visualization, and simulation control.
* Demo support with quick-load network examples.
* Iteration history and step-by-step replay.

---

## 🛠 Technologies Used

| Technology | Purpose |
| :--- | :--- |
| C++ | Core implementation language. |
| Raylib | GUI rendering and input handling. |
| Graph algorithms | Network modeling and optimization. |

---

## 🚀 Getting Started

### Prerequisites

* A C++ compiler with C++11 support or newer.
* Raylib installed and configured for your environment.
* Git, if cloning the repository.

### Clone the repository

```bash
git clone https://github.com/RounakChoudhary/DSA-PROJECT.git
cd DSA-PROJECT/DSA-project
```

### Build on Windows

From the `DSA-project` folder, use:

```powershell
g++ main.cpp -o DSA-project.exe -lraylib -lopengl32 -lgdi32 -lwinmm -luser32 -lkernel32
```

If your Raylib setup differs, adjust the library flags accordingly.

### Run the application

```powershell
.\\DSA-project.exe
```

### Quick start

1. Enter city names separated by commas, or use a demo button.
2. Click `Input Edges` and add network connections.
3. Select source and target cities, enter a data amount, and begin the simulation.
4. Observe the graph visualization and optimization output.

---

## 🧪 Demo modes

* `Load 5 cities demo` — quick small network example.
* `Load 50 cities demo` — deterministic 50-city network with preconfigured edges.

---

## 📁 Project structure

* `DSA-project/main.cpp` — app entry point, GUI, and main logic.
* `DSA-project/Network.h` — network graph model definitions.
* `DSA-project/CongestionManager.h` — congestion and flow algorithms.
* `DSA-project/GUI.h` — UI state, input handling, and drawing.
* `DSA-project/MaxCapacitySolver.h` — path finding and capacity optimization.
* `DSA-project/raylib.h` — Raylib header wrapper.

---

## 👥 Contributors

| Name | Contribution | GitHub |
| :--- | :--- | :--- |
| Akshaya | System definition and UI shell. | [@Akkiiitj](https://github.com/Akkiiitj) |
| Rounak | Max-flow implementation, visualization, and demo mode. | [@RounakChoudhary](https://github.com/RounakChoudhary) |
| Payal | Graph structure input and data panel handling. | [@payalt2006](https://github.com/payalt2006) |
| Likhita | Augmented path heuristic and iteration history controls. | [@likhita666](https://github.com/likhita666) |

---

## 📄 License

This project is licensed under the **MIT License**.
