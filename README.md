# Simulation-of-an-Autonomous-Robot-in-a-Warehouse

## Introduction

This is an implementation of a simulation for an autonomous robot navigating a rectangular warehouse. The objective of the simulation is to model the robot's movement from a starting position to a designated destination while avoiding obstacles. The warehouse is represented as a 10x10 grid, where the robot's navigation is governed by specific constraints and an efficient pathfinding algorithm.


## How to run the Simulation

Download the file and extract the contents.

Installing the requirements

    pip install -r requirements.txt

Running the Simulation

    python simulation.py

## Objectives

1.	Pathfinding: Implement an algorithm that allows the robot to navigate from the start to the destination while avoiding obstacles.

2.	Visualization: Provide a real-time visual representation of the robot's movement, including the starting point, destination, and obstacles.

## Approach

1. Environment Setup

The warehouse is modeled as a 10x10 grid, where each cell represents a unit of space (1 meter by 1 meter). The grid uses:
•	0 to indicate free space.
•	1 to represent obstacles.
The robot is initialized at the starting coordinates (0, 0) and is tasked to reach the destination at (7, 9).

2. Obstacle Configuration

Obstacles within the warehouse are defined through a 2D array structure. Users can either predefine the obstacle layout or allow the program to randomly generate them (if not specified). The robot is programmed to ensure that neither the starting point nor the destination overlaps with any obstacles.

3. Pathfinding Algorithm

The A* algorithm was implemented to determine the optimal path for the robot. This algorithm is well-suited for grid-based pathfinding as it combines the advantages of uniform-cost search and heuristics. The main steps of the algorithm include:
•	Initialization: Open and closed lists are established. The open list contains nodes to be evaluated, while the closed list contains nodes already evaluated.
•	Cost Calculation: For each movement, the algorithm calculates the cost of traveling to neighboring nodes, considering both the distance and any penalties for obstacles.
•	Path Reconstruction: Once the destination is reached, the path is reconstructed by tracing back from the destination to the start.

4 Robot Movement Simulation

The robot moves according to the following constraints:
•	It travels at a speed of 0.1 m/s.
•	After every 0.1 seconds of travel, it pauses for 2 seconds.
•	The robot can only move along the cardinal directions (up, down, left, right) and not diagonally.

The robot's movement is visualized in real-time, with:
•	The starting point marked in blue.
•	The destination marked in red.
•	The previous positions of the robot shown in gray.
•	Obstacles depicted as gray boxes.

5. Visualization
Using Matplotlib, the simulation provides a graphical representation of the warehouse. The visualization updates dynamically to reflect the robot's position after each movement, including the appropriate delays for both movement and pauses.


## Implementation Details

The following functions encapsulate the main components of the simulation:
•	create_warehouse(): Initializes the warehouse grid with the specified obstacle configuration.
•	AStar(): Implements the A* pathfinding algorithm to compute the path from start to destination.
•	visualize_movement(): Manages the real-time visualization of the robot's movement through the warehouse.

## Conclusion
The simulation effectively models the movement of an autonomous robot in a warehouse environment, adhering to specified constraints and successfully avoiding obstacles. The implementation of the A* algorithm ensures that the robot finds the optimal path, while the visualization provides a clear representation of its navigation.
