# ARP — Assignments Overview

This repository contains the three ARP assignments, organized as progressive steps of the same project.  
Each assignment folder includes its own updated and detailed README describing the specific work, design choices, and implementation details.

This README provides an overview of the evolution of the project and highlights the main differences between assignments.  
Older assignments can be considered the “history” of the project, while the latest one represents the most complete and refined version.

---

## ASSIGNMENT 1
Main implemented elements:
- **Blackboard architecture**
- **Window resizing functions**
- **Processes and pipes** for:
  - input (keyboard)
  - obstacles
  - targets
  - drone dynamics
- **Drone movement** based on physics forces and communication with the blackboard
- **Repulsive forces** from obstacles and the “fence”

---

## ASSIGNMENT 2
### Changes from Assignment 1:
- Renamed `world_physics` to `drone_physics` to better reflect its role
- Added additional checks on `fork()`, `pipe()`, `read()`, and `write()`
- Improved the **swirl component** of tangential forces (more stable behaviour)
- Clearer distinction between **collision radius** and **position radius**
- Introduced **sub-stepping integration** to avoid tunneling
- Improved position correction with a **stronger border clamp** (`0.001` to `0.1`)

### New elements:
- **Watchdog process**
- **Logfile system**
- **Process file**
- **Score system**

---

## ASSIGNMENT 3
### Changes from Assignment 2:
- Added the `current_index_target` variable to have the targets in sequence

### New elements:
- **Protocol**
