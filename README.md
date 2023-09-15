# COP3530 Maze Solver

## Overview

The COP3530 Maze Solver is a Python project that utilizes the Pygame library to generate and solve mazes. The application offers a graphical interface where users can interact with buttons to generate mazes, solve them using different algorithms like A* and Dijkstra's, and view various metrics like nodes expanded, path length, and time taken to solve.

## Prerequisites

- Python 3.x
- Pygame
- NumPy

## Installation

To install the required packages, run the following commands:

```bash
pip install pygame numpy
```

## Components

### DisjointSet Class

This class is utilized for Kruskal's algorithm to create the maze. It performs `find` and `union` operations.

### Maze Class

This class handles the maze generation, solving, and visualization. It contains methods for:

- Maze generation using Kruskal's algorithm
- Solving the maze using A* and Dijkstra's algorithms
- Resetting and displaying current stats

### User Interface

The interface offers buttons for:

- Maze generation
- Maze solving (A* and Dijkstra's)
- Clearing the maze
- Displaying stats
- Exiting the application

## Usage

1. Run the main script to start the application.
```bash
python app.py
```
2. Use the buttons in the graphical interface to interact with the application.

## Metrics Displayed

- Nodes Expanded: The number of nodes that were expanded during the solving process.
- Path Length: The length of the solution path from start to goal.
- Time to Solve: The time taken to solve the maze.
- Algorithm Used: The algorithm used to solve the maze (A* or Dijkstra's).

## Additional Notes

The application supports various maze dimensions and offers a way to visualize the maze-solving process.

