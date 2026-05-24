# ECE 276B Project 2: Motion Planning

## Overview

This project implements a motion planner for static and dynamic goals in continuous 3-D environments. The robot is modeled as a point moving inside a rectangular boundary with rectangular obstacle blocks. The goal is to compute collision-free paths from a start position to a goal position while avoiding all obstacles.

The implementation uses RRT* with continuous line-segment collision checking against axis-aligned bounding boxes (AABBs). For dynamic-goal environments, the planner reuses the RRT* tree across multiple goal queries instead of restarting from scratch for each new goal. This allows later goals to use previously explored regions of free space and improves replanning speed.

## Files

### main.py

This file contains the main testing, evaluation, and visualization code. It loads the map files, calls the planner, checks path validity, records performance metrics, and generates plots.

Static environment tests:
- test_flappy_bird()
- test_maze()
- test_monza()
- test_single_cube()
- test_tower()

Dynamic environment tests:
- experiment_window_dynamic()
- experiment_room_dynamic()

The file also includes helper functions for:
- plotting 3-D paths through the environment
- plotting XY, XZ, and YZ plane projections
- visualizing obstacle blocks in the plane projections
- running parameter sweeps over RRT* settings
- running timing sweeps for dynamic targets
- printing result rows for report tables

### Planner.py

This file contains the RRT* planner implementation.

The planner includes:
- goal-biased random sampling
- nearest-neighbor search using scipy.spatial.cKDTree
- steering with a fixed step size epsilon
- continuous line-segment collision checking against AABBs
- RRT* best-parent selection
- RRT* rewiring
- path extraction by backtracking through parent pointers
- tree reuse for dynamic replanning

Each node stores:
- coord: the 3-D configuration
- parent: the previous node in the tree
- g: the cost-to-come from the start

For static planning, the tree is initialized from the start node and expanded until the goal is reached or the maximum iteration limit is reached.

For dynamic planning, the same planner object is reused across the sequence of goals. The planner keeps self.nodes, self.coordinates, and self.tree so later goals can reuse the previously sampled tree.

### maps/

This folder contains the seven test environments. Each map is represented by a rectangular outer boundary and a list of rectangular obstacle blocks.

The environments are:
- E1_flappy_bird.txt
- E2_maze.txt
- E3_monza.txt
- E4_single_cube.txt
- E5_tower.txt
- E6_window.txt
- E7_room.txt

Environments E1-E5 are static-goal environments. Environments E6-E7 are dynamic-goal environments where the target moves through eight goal positions.

## How to Run

Run the project from the main project directory:

python3 main.py

The tests that run depend on what is enabled in the if __name__ == "__main__": block.

Example:

if __name__ == "__main__":
    test_flappy_bird()
    test_maze()
    test_monza()
    test_single_cube()
    test_tower()

    experiment_window_dynamic()
    experiment_room_dynamic()

    plt.show(block=True)

## Parameters

The main RRT* parameters are:
- epsilon = 0.5
- goal_bias = 0.1
- max_iter = 100000

where:
- epsilon is the steering distance
- goal_bias is the probability of sampling the goal directly
- max_iter is the maximum number of RRT* iterations

The final reported visualizations use:
- epsilon = 0.5
- goal_bias = 0.1
- max_iter = 100000

The parameter sweep also tested:
- epsilon in [0.25, 0.5, 1.0]
- goal_bias in [0.05, 0.1, 0.2]
- max_iter in [10000, 50000, 100000]

## Collision Checking

Collision checking is done continuously along each candidate edge. Each edge is treated as a line segment between two 3-D points, and each obstacle is treated as an axis-aligned bounding box.

The implementation uses a slab-based segment-AABB intersection test. A candidate edge is rejected if the line segment intersects any obstacle block. This is important because checking only the endpoints is not sufficient: two endpoints can be collision-free while the segment between them passes through an obstacle.

## Static Planning

For static environments E1-E5, the planner computes one path from the given start position to the fixed goal.

The planner reports:
- success or failure
- path length
- number of explored nodes
- number of iterations
- runtime

The final paths are visualized using both 3-D plots and 2-D plane projections.

## Dynamic Replanning

For dynamic environments E6-E7, the target moves through eight goal positions. The planner computes a path from the same start position to the current goal before moving to the next goal.

Instead of rebuilding the tree from scratch for each goal, the planner reuses the existing RRT* tree. Since the obstacle map is fixed, previously sampled collision-free nodes and edges remain valid. This makes later planning queries faster when the tree has already explored useful parts of the environment.

The dynamic experiments report:
- path success
- timing success
- path length
- runtime
- cumulative runtime
- explored nodes
- iterations

Timing sweeps are also included to test different values of t1 and t2, where t1 is the time available for the first goal and t2 is the time available for each later goal.

## Outputs

The code can generate:
- 3-D path visualizations
- XY, XZ, and YZ plane projections
- static parameter sweep results
- dynamic per-goal replanning results
- dynamic timing sweep results
- report-ready table rows

Figures are saved in the figures/ folder when the save functions are used.

## Dependencies

The project uses:
- numpy
- matplotlib
- scipy

Install dependencies with:

pip install numpy matplotlib scipy

## Run Instructions
cd starter_code
python3 main.py
