# 🌐 Network Optimization: Data Congestion & Cheapest Path Solver (DSA-PROJECT)

## 🌟 Project Overview

This project is a Data Structures and Algorithms (DSA) implementation designed to model and optimize a computer network. The primary goal is to *minimize data congestion* and determine the *cheapest (most optimal) path* for transporting data between any two branches (nodes) within the network.

It achieves this by leveraging the power of *graph data structures* and advanced graph algorithms.

## ✨ Features

The application provides a robust set of features for network modeling and optimization:

* *Max Flow/Min Cut Implementation:* Utilizes the *Edmonds-Karp algorithm* and an *Augmented Path Search Heuristic* to accurately calculate maximum data flow and identify network bottlenecks.
* *Cheapest Path Finding:* Implements pathfinding algorithms (like Dijkstra's) to find the path with the lowest overall "cost" between two points.
* *Interactive Graphical User Interface (GUI):* A visual interface built with *Raylib* displays network topology, congestion calculations, and includes iteration history controls.
* *Core Application Logic:* Robust data handling and structure initialization (Network.h, CongestionManager.h) providing a stable foundation for complex algorithms.

---

## 🛠 Technologies Used

| Technology | Purpose |
| :--- | :--- |
| *C++ / C* | Core implementation language for algorithms and data structures. |
| *Raylib* | Used for creating the graphical user interface (GUI) and visualization. |
| *Graph Data Structure* | Core data model for representing the network topology. |
| *Edmonds-Karp Algorithm* | Fundamental algorithm for solving the Max-Flow problem. |
| *Augmented Path Heuristic* | Used for efficient path finding within the Max-Flow computation. |

---

## 🚀 Getting Started

### Prerequisites

You will need a C++ compiler (like g++ or Clang) and the Raylib library installed on your system to compile and run this project.

* A C++ compiler supporting C++11 or later.
* The *Raylib* library (ensure it is configured correctly for your environment).

### Installation

1.  *Clone the repository:*
    bash
    git clone [https://github.com/RounakChoudhary/DSA-PROJECT.git](https://github.com/RounakChoudhary/DSA-PROJECT.git)
    cd DSA-PROJECT
    

2.  *Compilation:*
    Assuming you have raylib correctly linked, compile the project using your compiler. A common compilation command for Raylib projects is:
    bash
    # Example using g++ (adjust flags based on your Raylib setup)
    g++ main.cpp -o NetworkSolver -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
    

3.  *Run the application:*
    bash
    ./NetworkSolver
    

---

## 👥 Contributors

This project was a collaborative effort. The specific contributions for each team member are detailed below:

| Name | Focus Area & Key Deliverables | GitHub |
| :--- | :--- | :--- |
| *Akshaya* | *System Definition & UI Shell:* Defined and finalized the core application logic within the CongestionManager.h and established the foundational graphical framework for the application. | [@Akkiiitj](https://github.com/Akkiiitj) |
| *Rounak* | *Max-Flow & Visualization:* Implemented the full *Edmonds-Karp algorithm* and developed the critical logic for congestion calculation and network visualization component in the GUI. | [@RounakChoudhary](https://github.com/RounakChoudhary) |
| *Payal* | *Structure & Data Handling:* Initialized the fundamental *graph structure* and algorithm calling mechanisms, while building the necessary *Input and Output Data Panels* for the GUI. | [@payalt2006](https://github.com/payalt2006) |
| *Likhita* | *Heuristic & Iteration Control:* Implemented the essential *Augmented Path Search Heuristic* for Max-Flow and integrated the *Iteration History Display and Controls* into the UI. | [@likhita666](https://github.com/likhita666) |

---

## 📄 License

This project is licensed under the **[MIT License](https://opensource.org/licenses/MIT)** - see the LICENSE.md file (if one exists) for details.
