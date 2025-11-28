#ifndef WORLD_PHYSICS_H
#define WORLD_PHYSICS_H

#include "map.h"

//drone dynamics
void add_direction(GameState *g, int mx, int my);
void use_brake(GameState *g);
void add_drone_dynamics(GameState *g);

//repulsive forces
//void add_obstacles_repulsion();
//void add_fence_repulsion();

//attraction force
//void add_target_attraction();

#endif