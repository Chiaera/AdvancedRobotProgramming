# Assignment 1

This project implements an **interactive multi-process drone simulator** based on a **Blackboard architecture**, where multiple external processes communicate through Unix pipes.

---

## System Architecture

The system follows a **Blackboard architectural** pattern where multiple autonomous processes communicate with a central server (Blackboard):

![Architecture](img/architectures.jpg)

### Principal Components

| Component | Role | Process | Communication |
|----------|------|---------|----------------|
| **B: Blackboard Server** | - Central game server<br>- Physics updates<br>- Rendering<br>- Message routing | `blackboard` | Pipes + `select()` |
| **I: Input Manager** | - Captures keyboard input<br>- Sends directional commands | `process_input` | `pipe_input → Blackboard` |
| **D: Drone Dynamics** | Sends periodic tick messages (50 Hz) | `process_drone` | `pipe_drone → Blackboard` |
| **T: Target Generator** | Generates random target positions | `process_targets` | `pipe_targets → Blackboard` |
| **O: Obstacle Generator** | Generates random obstacle positions | `process_obstacles` | `pipe_obstacles → Blackboard` |

<br>

---

## Component Details & Execution Flow
1. Initialization
  When the simulator starts, the **Blackboard server** boots first.
  It performs the following actions:
    - Creates IPC channels using `pipe()`.
    - Spawns child processes (`process_input`, `process_drone`, `process_targets`, `process_obstacles`) using `fork()` and `execlp()`.
    - Initializes the shared `GameState` structure (map limits, parameters, default drone state)
    - Reads **simulation parameters** from `parameters.config` (world size, initial position of the drone, number of obstacles and targets, mass M, drag K, obstacle influence radius ρ, repulsion gains η)
    - Loads map and configuration files via the **Map Loader** (`map.c`), which uses `fopen()`, `fscanf()`, and updates initial fields of the GameState
   
   At this stage, all processes are running and ready to send data to the Blackboard

3. Message Flow During Simulation
  The simulator now enters its main operational phase
  Each external component contributes information to the Blackboard through a **non-blocking, asynchronous message flow**.
    * Input Manager → Blackboard
      - runs an ncurses non-blocking input loop (`nodelay()`, `getch()`)
      - each keystroke is converted into a control force message
      - sends messages through write(`pipe_input`)
      
      **Effect on flow:**<br>The Blackboard updates the *command force* stored in `GameState`.
      
     * Drone Process → Blackboard
       - acts as the global **timekeeper**,
       - uses `nanosleep()` to trigger a tick every 20 ms (50 Hz)
       - sends a **DRONE_TICK** message via write(`pipe_drone`)
       
       **Effect on the flow:**<br>The Blackboard, upon receiving a tick, it is used to trigger the **Physics Engine**.
      
    * Targets and Obstacles → Blackboard
      Both generator processes:
      - use `rand()` to compute new positions
      - send updates asynchronously using their respective pipes
      
      **Effect on flow:**<br>The Blackboard merges these updates into GameState.targets and GameState.obstacles.
    
  4. Blackboard Main Loop (Central Flow Control)
     The Blackboard runs a continuous loop driven by:
     - `select()`: monitors all pipes simultaneously
     - `read()`: retrieves available messages without blocking
     - update functions: apply received data into `GameState`
     - `drone_physics_step()`: computes new physics state
     - ncurses: refreshes the visual interface
       
     This loop is the core “heartbeat” of the system.
     It ensures coordination without requiring components to communicate directly with each other.

  5. Physics Update (Triggered by Drone Tick)
     Whenever a 20 ms tick arrives, the Blackboard calls the Physics Engine.
     The engine:
     - reads current position + velocity from `GameState`
     - compute all the forces (command force, obstacle repulsion and fence repulsion)
     - writes updated drone state back into `GameState`
     
     No IPC is used here: everything is in-memory within the Blackboard.

  6. Rendering Phase
     After physics is updated, the Blackboard:
     - draws the map
     - draws the drone, targets, obstacles
     - prints HUD elements (velocity, forces, position) using classic ncurses primitives.
       
     This creates an interactive console-based simulation refreshed at ~50Hz.
<br>

### Shared Component: GameState
Although not an active process, `GameState` is the central shared structure that:
- stores all world data
- receives updates from all external modules
- is read by the physics engine and renderer

It is declared in `world.h` and acts as the “memory” of the Blackboard.
  
<br>
   
---

## Physics Model

The drone motion follows the dynamic equation:

$$F = M \frac{d^2 p}{dt^2} + K \frac{dp}{dt}$$

Where:
- **p** = drone position (x, y)  
- **M** = mass  
- **K** = viscous drag coefficient
- **ΣF** = total force (command + obstacle + fence)

### Forces implemented

1. **Command Force (F<sub>cmd</sub>)**  
   Updated incrementally from input commands (8 directions + brake).<br>
   **Key Functions:**
    |Key|Action|Description|
    |---|---|---|
    |w or W|Move Up-Left|Diagonal movement|
    |e or E|Move Up|Vertical movement|
    |r or R|Move Up-Right|Diagonal movement|
    |s or S|Move Left|Horizontal movement|
    |d or D|Brake|Reduces velocity <br>and force by 50%|
    |f or F|Move Right|Horizontal movement|
    |x or X|Move Down-Left|Diagonal movement|
    |c or C|Move Down|Vertical movement|
    |v or V|Move Down-Right|Diagonal movement|
    |q or Q|Quit|Shutdown simulator|

2. **Obstacle Repulsion (F<sub>obst</sub>)**  
   Modified Khatib potential field with radial and tangential components:
   
   $$F_{\text{rep}} = \eta\left(\frac{1}{d} - \frac{1}{\rho}\right)\frac{1}{d^2},  \qquad d < \rho$$
   
   Where
      - **ρ (rho)**: influence radius of obstacles
      - **η (eta)**: radial repulsion gain
      - **d**: distance from obstacle center

   Tangential force creates a smooth “swirling” effect around obstacles.

4. **Fence Repulsion (F<sub>fence</sub>)**  
   Avoids boundary collisions by pushing the drone away from the world limits.

<br>

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
│ └── drone_physics.h
├── src/
│ ├── blackboard.c
│ ├── map.c
│ ├── process_drone.c
│ ├── process_input.c
│ ├── process_obstacles.c
│ ├── process_targets.c
│ ├── world.c
│ └── drone_physics.c
├── Makefile
└── README.md
```

<br>

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

### Launch
The `MakeFile` is responsable for removing the previous builds and compile all the files, so you can directly run the program.
```bash
#from the assignment1 directory
cd ~/AdvancedRobotProgramming/assignment1
make run
```
