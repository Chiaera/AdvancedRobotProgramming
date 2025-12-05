# Assignment 1 – Drone Operation Simulator

This project implements an **interactive multi-process drone simulator** based on a **Blackboard architecture**, where several external processes communicate through Unix pipes to update the global GameState.

---

## System Architecture

Below is the architecture diagram showing how each process interacts with the central Blackboard:

![Architecture](img/architectures.jpg)

### Components Overview

| Component | Role | Files | Communication |
|----------|------|-------|----------------|
| **B: Blackboard Server** | Main process. Loads parameters, maintains GameState, updates physics, draws the map, receives messages. | `blackboard.c`, `map.c`, `world.c`, `world_physics.c` | Pipes + `select()` |
| **I: Input Manager** | Handles keyboard input and sends directional/brake commands. | `process_input.c` | `pipe_input → Blackboard` |
| **D: Drone Dynamics** | Sends periodic tick messages to trigger physics updates. | `process_drone.c` | `pipe_drone → Blackboard` |
| **T: Target Generator** | Generates initial random target positions. | `process_targets.c` | `pipe_target → Blackboard` |
| **O: Obstacle Generator** | Generates initial random obstacle positions. | `process_obstacles.c` | `pipe_obstacles → Blackboard` |
| **Physics Engine** | Computes forces and updates drone motion via Euler integration. | `world_physics.c` | internal |
| **Map Renderer** | Draws the world, drone, obstacles, and targets using ncurses. | `map.c` | internal |

---

## Physics Model

The drone motion follows the dynamic equation:
\[
\sum F = M \frac{d^2 p}{dt^2} + K \frac{dp}{dt}
\]

Where:
    - **p** = drone position (x, y)  
    - **M** = mass  
    - **K** = viscous drag  
    - **ΣF** = total force (command + obstacle + fence)

### Forces implemented

1. **Command Force (F<sub>cmd</sub>)**  
   Updated incrementally from input commands (8 directions + brake).

2. **Obstacle Repulsion (F<sub>obst</sub>)**  
   Based on a modified Khatib potential field, with:
   - radial repulsion  
   - tangential (swirl) component  

3. **Fence Repulsion (F<sub>fence</sub>)**  
   Avoids boundary collisions by pushing the drone away from the world limits.

Position is stored as **double precision** internally to preserve subcell movement.

---

## Repository Structure
The project is structured as follows:
```bash
assignment1/
│
├── bin/
│ └── parameters.config
├── build/
│ └── bin/ (compiled executables)
├── img/
│ └── architectures.jpg
├── include/
│ ├── map.h
│ ├── process_drone.h
│ ├── process_input.h
│ ├── world.h
│ └── world_physics.h
├── src/
│ ├── blackboard.c
│ ├── map.c
│ ├── process_drone.c
│ ├── process_input.c
│ ├── process_obstacles.c
│ ├── process_targets.c
│ ├── world.c
│ └── world_physics.c
├── Makefile
└── README.md
```

---

## Build & Run

### Prerequisites
- `gcc`
- `make`
- `ncurses` library
- `konsole` (used to spawn subprocesses automatically)

### Clone the repository
```bash
gh repo clone Chiaera/AdvancedRobotProgramming
```

### Compile and run
```bash
#from the assignment1 directory
cd ~/AdvancedRobotProgramming/assignment1
make run
```
