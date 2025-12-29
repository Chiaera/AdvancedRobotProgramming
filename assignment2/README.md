# Assignment 2

This project implements an **interactive multi-process drone simulator** based on a **Blackboard architecture**, where multiple external processes communicate through Unix pipes.

---

## System Architecture

![Architecture](img/architectures.png)


### Components details

The system implements **6 active processes** as required by the assignment specification. They represent a **Blackboard architectural** pattern where multiple autonomous processes communicate with a central server (Blackboard):
| # | Component | Role | Communication |
|---|-----------|-------|---------------|
| 1 | **Blackboard Server** | - Central game server<br>- physics engine<br>- rendering | Pipes + `select()` |
| 2 | **Input Manager** | - Captures keyboard input<br>- Sends directional commands | `pipe_input` |
| 3 | **Drone Process** | Sends periodic tick messages (50 Hz) | `pipe_drone` |
| 4 | **Targets Generator** | Random target spawner | `pipe_targets` |
| 5 | **Obstacles Generator** | Generates random obstacle positions | `pipe_obstacles` |
| 6 | **Watchdog** | System monitor | Shared memory + signals |

#### 1. Blackboard server
The Blackboard is the global server, it runs a continuous loop driven by:
  - `read()` the parameters file
   - update functions: apply received data into `GameState`
   - update the physics by calling `drone_physics()`
   - ncurses: refreshes the visual interface
   - monitors all pipes simultaneously with `select()`
       
     It ensures coordination without requiring components to communicate directly with each other.

#### 2. Input manager
It is responsable for update the *command forces* stored in `GameState`:
  - get input from keyboard
  - convert the input into a control force message 

#### 3. Drone process
It acts as the global **timekeeper**,
  - uses `nanosleep()` to trigger a tick every 20 ms (50 Hz) for update the **physics engine**
  - sends a **DRONE_TICK** message via write(`pipe_drone`)

#### 4. Target generator
It is uded to spawn the target every 30 seconds
  - use `rand()` to compute new positions
  - - send updates asynchronously using their respective pipes

#### 5. Target generator
Similar to the **Target process**, it is uded to spawn the target every 30 seconds
  - use `rand()` to compute new positions
  - - send updates asynchronously using their respective pipes

#### 6. Watchdog
It is used to monitor all the system:
  - managment the heartbeat table
  - check the status of the processes

Instead, the shared component are
  - #### GameState file
      It is a central shared structure, containts in `world` file where all the variables are contained. All processes can read the values by the header.

  - #### Map loader
    `map` is responsable for update the parameters and upload the map.
  - #### Shared Heartbeat Memory
    It is impliemented by `heartbeat` process and consider a safety control with the use of a semaphore.


---
## Execution Flow
### 1. Initialization
**Blackboard** 
  - creates the pipes and the child process with `fork()` and `execlp()` -> active all the processes
  - initializes the GameState
  - read the parameters from `parameters.config`
  - upload the map

### 2. Message from process to blackboard (without blocking the ncurses window)
**Intput** -> **Blackboard**
  - convert the keypress in the *command force*
  - update the force variables `fx_cmd` and `fy_cmd` in `GameState`
    
###**Drone** -> **Blackboard**
  - send tick every 20ms
  - update the `drone_physics`

###**Targets** -> **Blackboard** and **Obstacles** -> **Blackboard**
  - send tick every 30ms
  - respawn the target (or obstacle)
  - upload the `GameState` position

### Blackboard loop
  - monitoring pipes with `select()`
  - `read()` the messages and update the Gamestate with the new values
  - update the `drone_physics` after the relative tick
  - `respaw_obstacles()` or `respaw_targets` after receiving the relative tick
  - rendering the ncurses window

### Physics 
  - compute the forces: command, obstacles repulsion, fence repulsion and collision
  - impliemed the sub-stepping method
  - update the GameState

### Rendering
  - draw map
  - draw drone, targets and obstacles
  - update velocities, forces and positions

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
assignment2
      ├── bin
      │   └── parameters.config
      ├── img
      │   ├── architectures.png
      │   ├── collision.png
      │   └── screenshot.png
      ├── include
      │   ├── drone_physics.h
      │   ├── heartbeat.h
      │   ├── logger.h
      │   ├── map.h
      │   ├── process_drone.h
      │   ├── process_input.h
      │   └── world.h
      ├── Makefile
      ├── README.md
      └── src
          ├── blackboard.c
          ├── drone_physics.c
          ├── map.c
          ├── process_drone.c
          ├── process_input.c
          ├── process_obstacles.c
          ├── process_targets.c
          ├── watchdog.c
          └── world.c

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
